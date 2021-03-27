#ifndef _MAIN_PROCESS_H_
#define _MAIN_PROCESS_H_

#include "Arduino.h"



#define	EXTIO_NUM			2 //8個IO為一組
#define	INPUT_8_NUMBER		2
#define OUTPUT_8_NUMBER		1

#define	OUTPUT_NONE_ACTIVE	0
#define	OUTPUT_ACTIVE		1

#define	INPUT_NONE_ACTIVE	0
#define	INPUT_ACTIVE		1

#define WORKINDEX_TOTAL 	4
#define BUZZ				48

typedef struct _DigitalIO_
{
	uint8_t	Input[4];
	uint8_t	Output[4];
	uint8_t PreOutput[4];
}DigitalIO;

typedef struct _MainDataStruct_
{
//	此處的變數值會寫到EEPROM
	char 		Vendor[10];
	uint8_t 	HMI_ID;
	int			TestMaindataValue;
}MainDataStruct;


typedef struct _RuntimeStruct_
{
//	此處為啟動後才會使用的變數
	int  	Workindex[WORKINDEX_TOTAL];
	int		preWorkindex[WORKINDEX_TOTAL];
	
	uint8_t sensor[INPUT_8_NUMBER*8 + EXTIO_NUM*8];
	uint8_t outbuf[(OUTPUT_8_NUMBER+EXTIO_NUM)*8];

	bool 		UpdateEEPROM;
	int			TestRuntimedataValue;
}RuntimeStatus;

void setOutput(uint8_t index, uint8_t hl);
uint8_t getInput(uint8_t index);


void MainProcess_Task();
void MainProcess_Init();
void buzzerPlay(int playMS);

#endif	//_MAIN_PROCESS_H_

