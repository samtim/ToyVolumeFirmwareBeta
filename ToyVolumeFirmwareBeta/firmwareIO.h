#ifndef _FIRMWARE_IO_H_
#define _FIRMWARE_IO_H_

#include<SPIFlash.h>
#include<SPI.h>

//the pcb has 3 areas set aside for IO.  Area 0, 1, and 2.
//this file contains the methods and variables that handle the io to those memory on the pcb.

 //input from the other side of the serial port


 //=============================SPI flash variable==========================//
#define WP  10
#define wp_off     digitalWrite(WP,HIGH)
#define wp_on      digitalWrite(WP,LOW)

SPIFlash flash;
//==================================================================//

bool writeFlag = false;
bool readFlag = false;

char rChar;
uint32_t address;
unsigned long timeDelta=0;
uint32_t startAddress = 0; //just for keeping track of lengths.

void startWrite(uint32_t block) //prepare to record input to us and write it to the pcb memory
{
    if (readFlag)
    {
      Serial.println("::fail::done");
      readFlag = false;
    }
    if (writeFlag)
      return;
    
    wp_off;   //turn off write protect
    writeFlag=true;
    flash.eraseBlock64K(block,0);    //need to be erased before write, this should be the page number, it will erase the 256*256 bytes block which contain this page
    address=(uint32_t)block*256;
    Serial.println("");
}

void startRead(uint32_t _address) //prepare to read our pcb memory and send it to the serial port
{
  if (writeFlag)
  {
    Serial.println("recording::done::fail");
    writeFlag = false;
  }
  
  if (readFlag)
    return;
    
  readFlag = true;
  address = _address;
  Serial.println("");
}

//returns true if caller should reping the initialization
bool readSerial(void)
{
  bool strFlag = false;
  String inputStr = "";
  
  while (Serial.available()) //read any data coming to us
  {
    inputStr = Serial.readString();
    strFlag=true;
  }

  if (!strFlag)
    return false;

  if(inputStr=="write0") 
  {
    startWrite(0);
    Serial.println("mode::recording::0");
  }
  else if(inputStr=="write1") 
  {
    startWrite(256);
    Serial.println("mode::recording::1");
  }
  else if(inputStr=="write2") 
  {
    startWrite(512);
    Serial.println("mode::recording::2");
  }
  
    else if(inputStr=="read0") 
  {
    startRead(0);
    Serial.print("data0::");
  }
  else if(inputStr=="read1") 
  {
    startRead((uint32_t)256*256);
    Serial.print("data1::");
  }
  else if(inputStr=="read2") 
  {
    startRead((uint32_t)512*256);
    Serial.print("data2::");
  }
  else if(inputStr=="reping") //on osx, the chip does not actually reset unles unplugged.  To know we are talking to the right chip, we can simply send this string and if it comes back correctly we know we are live. 
  {
    Serial.println("");
    return true;
  }
  else 
  {
    //unkown command... ignore.
  }
  return false;
}


void updateSerialIn(void)
{
  unsigned long timeDelta=millis();
  startAddress = address;
  while(writeFlag)
  {
    while (Serial.available()) 
    {
      rChar = Serial.read();
      flash.writeByte(address,rChar,0);
      address++;
      timeDelta=millis();
    }
    //if no input from Serial for 2000ms, it will jump out of write mode
    if(millis()-timeDelta > 2000) 
    {
      writeFlag=false;  
      Serial.print("::recording::");
      Serial.print(address - startAddress);
      Serial.println("::done"); 
    }  
    wp_on; //turn on write protect after updating
  }
}

void updateSerialOut(void)
{
  startAddress = address;
  while(readFlag)
  {
      rChar=flash.readChar(address,0);
       
      // the memory is filled in with FF when erased, read as -1 by readChar
      if(rChar != -1) 
        Serial.print(rChar); //print out the next character
      else
      {
        readFlag=false;
        Serial.print("::");
        Serial.print(address - startAddress); //bytes written. 
        Serial.println("::done");
      }

      address++;
   }
}


#endif
