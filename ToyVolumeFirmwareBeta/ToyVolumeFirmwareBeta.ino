
/*
  Pin outs
  the FPC on the touch panel is 8 pins, pin 1 is to the right pin 8 to the left with the display facing up

  pin | function  | Arduino Mega
  --------------------------
  8   | NULL      | NULL
  7   | INT       | 19
  6   | SDA       | 20
  5   | SCL       | 21
  4   | 1.8V      | NULL
  3   | WAKE      | 18
  2   | VCC       | VCC
  1   | GND       | GND
*/


#include "Wire.h"
#include "Arduino.h"
#include "ToyVolumeTP.h"
#include "firmwareIO.h"
#define BIGFLASH

///////////////////////////////////////////////////////////////////////////////////////////
//UPDATE THIS WHENEVER YOU PUBLISH A CHANGE!
const static float firmwareVer = 4.5;           
///////////////////////////////////////////////////////////////////////////////////////////


static inline void  confirmInit()
{
  String firmwareStr = "firmwareVersion::";
  firmwareStr += "touchPanelsPCB::";
  firmwareStr += firmwareVer;
  Serial.println(firmwareStr);
  Serial.println("init::done");
}






unsigned int x, y, x1, y1;
unsigned char deltaX, deltaY;

#define GET_FAR_ADDRESS(var) \
  ({ \
    uint_farptr_t tmp; \
    \
    __asm__ __volatile__( \
                          \
                          "ldi %A0, lo8(%1)" "\n\t" \
                          "ldi %B0, hi8(%1)" "\n\t" \
                          "ldi %C0, hh8(%1)" "\n\t" \
                          "clr %D0" "\n\t" \
                          : \
                          "=d" (tmp) \
                          : \
                          "p" (&(var)) \
                        ); \
    tmp; \
  })



// Pins
#define WAKE 18
#define INTRPT 19
#define LED 2
#define PAUSE 10

#define SCREEN_MAX_X     1600
#define SCREEN_MAX_Y    960

#define GSLX680_I2C_ADDR  0x40

#define GSL_DATA_REG    0x80
#define GSL_STATUS_REG    0xe0
#define GSL_PAGE_REG    0xf0

#define delayus delayMicroseconds


#define DATA_SIZE 44    //change the DATA SIZE to 44 to support up to 10 track points on Sept 19



/////////////////////////////////////////
struct _coord {
  uint32_t x, y;
  uint8_t finger;
};

//Unity can handle multiple touch panels on a single device.
//For old software to be compatible with future hardware that may feature multiple touch panels Unity expects all their data to come from our single serial port.
//To distinguish between the touches of these different panels, the n_panelID is sent before each send.
/*if a device has multiple touch screens, this is the way to distinguish between them, send the appropriate n_panelID
        FRONT_TOUCHSCREEN = 0
        BACK_TOUCHSCREEN = 1
        LEFT_TOUCHSCREEN = 2 //relative to the device itself
        RIGHT_TOUCHSCREEN = 3 //relative to the device itself
        TOP_TOUCHSCREEN = 4
        BOTTOM_TOUCHSCREEN = 5

        OTHER touch screens are valid up to 7.  These will not have any spatial correspondence to the volume/hypercube 
        */
struct _ts_event
{
  uint8_t  n_panelID;  //what touch panel does this represent?  front ? then n_panelID = 0;
  uint8_t  n_fingers;
  struct _coord coords[10];
};

struct _ts_event ts_event;

uint8_t touch_data[DATA_SIZE] = {0};
int bufferSizeFromPCB = 0;

bool led = false;
////////////////////////////////////////



static inline void wiresend(uint8_t x) 
{
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

bool i2c_write(uint8_t reg, uint8_t *buf, int cnt)
{
  Wire.beginTransmission(GSLX680_I2C_ADDR);
  wiresend(reg);
  for (int i = 0; i < cnt; i++) {
    wiresend(buf[i]);
  }
  int r = Wire.endTransmission();
  if ((r != 0) && (r != 4)) {
    Serial.print("i2c write error: ");
    Serial.print(r);
    Serial.print(" ");
    Serial.println(reg, HEX);
  }
  return r == 0;
}

int i2c_read(uint8_t reg, uint8_t *buf, int cnt)
{
  Wire.beginTransmission(GSLX680_I2C_ADDR);
  wiresend(reg);
  int r = Wire.endTransmission();
  if (r != 0) {
    Serial.print("i2c read error: ");
    Serial.print(r);
    Serial.print(" ");
    Serial.println(reg, HEX);
  }

  int n = Wire.requestFrom(GSLX680_I2C_ADDR, cnt);
  if (n != cnt) {
    Serial.print("i2c read error: did not get expected count ");
    Serial.print(n);
    Serial.print(" - ");
    Serial.println(cnt);
  }

  for (int i = 0; i < n; i++) {
    buf[i] = wirerecv();
    //       Serial.print(buf[i]);
    //       Serial.print("-");
  }
  return n;
}

void clr_reg(void)
{
  uint8_t buf[4];

  buf[0] = 0x88;
  i2c_write(0xe0, buf, 1);
  delay(5);

  buf[0] = 0x01;
  i2c_write(0x80, buf, 1);
  delay(5);

  buf[0] = 0x04;
  i2c_write(0xe4, buf, 1);
  delay(5);

  buf[0] = 0x00;
  i2c_write(0xe0, buf, 1);
  delay(5);
}

void reset_chip()
{
  uint8_t buf[4];

  buf[0] = 0x88;
  i2c_write(GSL_STATUS_REG, buf, 1);
  delay(5);

  buf[0] = 0x04;
  i2c_write(0xe4, buf, 1);
  delay(5);

  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  i2c_write(0xbc, buf, 4);
  delay(5);
}

void load_fw(void)
{
  uint8_t addr;
  uint8_t Wrbuf[4];
  uint16_t source_line = 0;
  uint16_t source_len = sizeof(GSLX680_FW) / sizeof(struct fw_data);
  Serial.print("Firmware size: "); Serial.println(sizeof(GSLX680_FW));
  Serial.print("Line numbers : "); Serial.println(source_len);

  for (source_line = 0; source_line < source_len; source_line++) {

    addr = pgm_read_byte_far(GET_FAR_ADDRESS(GSLX680_FW[0].offset) + source_line * 5);

    Wrbuf[0] = (char) (pgm_read_dword_far(GET_FAR_ADDRESS(GSLX680_FW[0].val) + source_line * 5) & 0x000000ff);

    Wrbuf[1] = (char) ((pgm_read_dword_far(GET_FAR_ADDRESS(GSLX680_FW[0].val) + source_line * 5) & 0x0000ff00) >> 8);
    Wrbuf[2] = (char) ((pgm_read_dword_far(GET_FAR_ADDRESS(GSLX680_FW[0].val) + source_line * 5) & 0x00ff0000) >> 16);
    Wrbuf[3] = (char) ((pgm_read_dword_far(GET_FAR_ADDRESS(GSLX680_FW[0].val) + source_line * 5) & 0xff000000) >> 24);

    i2c_write(addr, Wrbuf, 4);
    //     Serial.print("+");
  }
}


void startup_chip(void)
{
  uint8_t buf[4];

  buf[0] = 0x00;
  i2c_write(0xe0, buf, 1);
}

void init_chip()
{
  Serial.println("Toggle Wake");
  digitalWrite(WAKE, HIGH);
  delay(PAUSE);
  digitalWrite(WAKE, LOW);
  delay(PAUSE);
  digitalWrite(WAKE, HIGH);
  delay(PAUSE);

  // CTP startup sequence
  Serial.println("clr reg");
  clr_reg();
  delay(PAUSE);

  Serial.println("reset_chip");
  reset_chip();
  delay(PAUSE);

  Serial.println("loading_fw");
  load_fw();
  delay(PAUSE);

  startup_chip();
  Serial.println("reset_chip2");
  reset_chip();
  Serial.println("startup_chip");
  startup_chip();

  confirmInit();
}


int read_data(void)
{
  //Serial.println("reading data...");
  touch_data[DATA_SIZE] = {0};
  bufferSizeFromPCB = i2c_read(GSL_DATA_REG, touch_data, DATA_SIZE);
  ts_event.n_fingers = touch_data[0];
  for (int i = 0; i < ts_event.n_fingers; i++) {
    ts_event.coords[i].x = ( (((uint32_t)touch_data[(i * 4) + 5]) << 8) | (uint32_t)touch_data[(i * 4) + 4] ) & 0x00000FFF; // 12 bits of X coord
    ts_event.coords[i].y = ( (((uint32_t)touch_data[(i * 4) + 7]) << 8) | (uint32_t)touch_data[(i * 4) + 6] ) & 0x00000FFF;
    ts_event.coords[i].finger = (uint32_t)touch_data[(i * 4) + 7] >> 4; // finger that did the touch
  }

  return ts_event.n_fingers;
}

void setup() {
  Serial.begin(57600);   // change the baud rate to 57600 to match SPI Write time
  Serial.println("Starting");
  pinMode(LED, OUTPUT);
  pinMode(WAKE, OUTPUT);
  digitalWrite(WAKE, HIGH);
  pinMode(INTRPT, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(400000);
  delay(PAUSE);
  init_chip();
  Serial.write(255);Serial.write(255); //delimiter for touch frames
  
//================init external flash=========//
  pinMode(WP,OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2,HIGH);
  wp_on;
  Serial.setTimeout(100);   //jump out if there is no input for 100 ms 
  flash.begin();
}

bool sentNullByte = false;
void updateTouchScreenOut()
{
  if (readFlag || writeFlag)
    return;
  
   if (digitalRead(INTRPT) == HIGH) 
  {
    sentNullByte = false;
    
    int n = read_data();

    //as bytes
   Serial.write(ts_event.n_panelID);
   Serial.write(ts_event.n_fingers);
    for(int i=0; i < ts_event.n_fingers; i++)
    {
      Serial.write(ts_event.coords[i].finger);
      Serial.write(ts_event.coords[i].x);
      Serial.write(ts_event.coords[i].x >> 8); //send as 16 bit
      Serial.write(ts_event.coords[i].y);
      Serial.write(ts_event.coords[i].y >> 8);
    }
    Serial.write(255);Serial.write(255); //delimiter for touch frames

    
  /*      
    //human readable
    Serial.print(n);
    for(int i=0; i<n; i++)
    {
      Serial.print(" ");
      Serial.print(ts_event.coords[i].finger); Serial.print(" "), Serial.print(ts_event.coords[i].x); Serial.print(" "), Serial.print(ts_event.coords[i].y);            
    }
    Serial.println("");
  */  

    //as raw touchscreen data
    //Serial.write(touch_data,bufferSizeFromPCB);  
       
    /*
    //binary debug
    for(int i=0; i < bufferSizeFromPCB; i++){
         Serial.print(touch_data[i]);
         Serial.print(" ");
     }
     Serial.println("");
    */

  }
  else if (!sentNullByte && ts_event.n_fingers == 0) //this ensures that we send at least 1 update to Unity telling it that all touches are up
  {
    Serial.write(0);Serial.write(255);Serial.write(255); 
    sentNullByte = true;
  }
}

void loop() 
{
  updateTouchScreenOut();

  if (readSerial()) //do we have any commands coming over to us? 
    confirmInit();

  updateSerialIn(); 
  updateSerialOut(); 
} 


