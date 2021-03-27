#ifndef _EEPROM_FUNCTION_H_
#define _EEPROM_FUNCTION_H_

#include "Arduino.h"
#include "EEPROM.h"

// 利用這兩個C++模板，讀寫整組資料
// 參數address是EEPROM位址，參數data是想讀寫的資料結構
template <class T> int EEPROM_writeAnything(int address, const T &data);
template <class T> int EEPROM_readAnything(int address, T &data);

// 函式模板的定義
// 使用EEPROM程式庫寫入一個一個的byte


void Clear_EEPROM();
void READ_EEPROM();
void WRITE_EEPROM();


#endif //_EEPROM_FUNCTION_H_

