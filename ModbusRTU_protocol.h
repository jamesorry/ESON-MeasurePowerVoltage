#ifndef _MODBUSRTU_PROTOCOL_H_
#define _MODBUSRTU_PROTOCOL_H_

#include "Arduino.h"
#include "cppQueue.h"
#include <SoftwareSerial.h>

#define MBRTU_CMD_DATA_MAX_LEN 		128
#define TIME_MBRTU_CMD_INTERVAL		500
#define MBRTU_CMD_RETRY_MAX			3

#define MBRTU_CMD_BYTE_ID			0
#define MBRTU_CMD_BYTE_FUNCTION		1
#define MBRTU_CMD_BYTE_Addr			2
#define MBRTU_CMD_BYTE_Data			4
#define MBRTU_CMD_BYTE				6


typedef struct{
	uint8_t datatype;
	uint8_t retrycnt;
	uint8_t datalen;
	uint8_t data[32];
} ModbusRTUCmdRec;

class ModbusRTU_Protocol
{
private:
	Queue	 *cmdQueue;		//!< Queue implementation: FIFO LIFO
	ModbusRTUCmdRec cmdRec;

	uint8_t	 ProcessIndex;
	uint16_t ProcessTimeCnt;
	
	Stream 	 *modbus_port = NULL;
	bool 	 SWSerial = false;
	
	uint16_t SendCmdTimeCnt = 0;
	String 	 Command;
public:
	bool     Update;
	uint8_t  recdata[MBRTU_CMD_DATA_MAX_LEN];
	uint8_t  reclen;

	ModbusRTU_Protocol(HardwareSerial* cmdport, const uint32_t baudrate=115200);
	ModbusRTU_Protocol(SoftwareSerial* cmdport, const uint32_t baudrate=115200);	
	~ModbusRTU_Protocol();

	void SendCommandQ(void);
	void Process(void);
	uint16_t ComputeCRC(uint8_t *buf, uint8_t length);
	bool Send_Command(uint8_t      ID,  uint8_t FunctionCode, uint32_t Addr, uint32_t Datacnt, uint32_t value);
	bool CheckReciveData();

	bool Set_RPM(uint32_t RPM);
	bool Set_Step(uint32_t Step);
	bool Absolute_position();
	bool Emergency_Stop();
	bool Search_Home();
};
#endif
