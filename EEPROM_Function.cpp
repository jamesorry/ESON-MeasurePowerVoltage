#include "EEPROM_Function.h"
#include "MainProcess.h"
#include "hmi.h"

#define EEPROM_DEBUG	0

extern MainDataStruct maindata;
extern HardwareSerial *cmd_port;


void READ_EEPROM()
{ 
// 讀取EEPROM，回傳值count代表總共寫入幾個byte 
 
  int count = EEPROM_readAnything(0, maindata);
  #if EEPROM_DEBUG
  	if(cmd_port != NULL)
	{
  		cmd_port->println("Read EEPROM.");
  		cmd_port->print("Vendor: ");
  		cmd_port->println(maindata.Vendor);
  	}
  #endif
  
  if (String(maindata.Vendor) != VENDOR)  //未定義 時寫入初值 NNNNNNNNNNNN  
  {
     Clear_EEPROM();
  }
  
//  Serial.println(KEEP_DATA.Vendor);
//  Serial.println(KEEP_DATA.RS485_ID);

//  Serial.print(count);
//  Serial.println(" bytes read.");
  

}

void WRITE_EEPROM()
{  
  // 寫入EEPROM，回傳值count代表總共寫入幾個byte 
  int count = EEPROM_writeAnything(0, maindata);
//  Serial.print(count);
//  Serial.println(" bytes written.");
//  Serial.println("---------------");
  
//	if(cmd_port != NULL) 
//		cmd_port->println("WRITE_EEPROM End.");
}


void   Clear_EEPROM()
{ 
	uint8_t i;
#if EEPROM_DEBUG	
  	if(cmd_port != NULL)
	{
  		cmd_port->println("Clear EEPROM.");
  	}
#endif
	strcpy(maindata.Vendor, VENDOR); 
	
// 寫入EEPROM，回傳值count代表總共寫入幾個byte 
   int   count = EEPROM_writeAnything(0, maindata);
     
}

template <class T> int EEPROM_writeAnything(int address, const T &data)
{
  const byte *p = (const byte *)(const void *)&data;
  int i, n;
  for(i = 0, n = sizeof(data); i < n; i++)
    EEPROM.write(address++, *p++);
  return i;
}
template <class T> int EEPROM_readAnything(int address, T &data)
{
  byte *p = (byte *)(void *)&data;
  int i, n;
  for(i = 0, n = sizeof(data); i < n; i++)
    *p++ = EEPROM.read(address++);
  return i;
}



