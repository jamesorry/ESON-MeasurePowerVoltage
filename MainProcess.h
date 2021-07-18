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

#define STATUS                  0

#define STATUS_STOP             0
#define STATUS_CHECK_BTN        1
#define STATUS_CHECK_START      2
#define STATUS_READ_VOLTAGE     3
#define STATUS_CHECK_VOLTAGE    4
#define STATUS_READ_WATTAGE     5
#define STATUS_CHECK_WATTAGE    6
#define STATUS_CHECK_END        7

#define ADC_V_0         13
#define ADC_V_Bus       11
#define ADC_V_15        9
#define ADC_V_3_3       3
#define ADC_V_5         5
#define ADC_V_12        7
#define Relay           17
#define InStartBtn      15  
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

	bool 	UpdateEEPROM;
    bool    UpdateHMIReceive = false;
	int		TestRuntimedataValue;
    int     VoltageReadData[6] = {0};
    int     WattageReadData = -1;
    float   V_0 = 0;
    float   V_Bus = 0;
    float   V_15 = 0;
    float   V_3_3 = 0;
    float   V_5 = 0;
    float   V_12 = 0;

    float   RealV_0 = 0;
    float   RealV_Bus = 0;
    float   RealV_15 = 0;
    float   RealV_3_3 = 0;
    float   RealV_5 = 0;
    float   RealV_12 = 0;
    int     ReadWattageRetryTimes = 0;
}RuntimeStatus;

void setOutput(uint8_t index, uint8_t hl);
uint8_t getInput(uint8_t index);


void MainProcess_Task();
void MainProcess_Init();
void ReadVoltage(uint8_t Num);
void ProcessStasus();

void buzzerPlay(int playMS);
void MainProcess_Timer(void);
bool CheckVoltageMsg();
bool CheckWattageMsg();
bool CheckStartEnd(uint8_t StartEnd);
bool CheckPing(void);
bool CheckBtn(void);

#endif	//_MAIN_PROCESS_H_

