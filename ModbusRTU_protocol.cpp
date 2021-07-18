#include "Arduino.h"
#include "ModbusRTU_protocol.h"
#include "hmi.h"
#include "cppQueue.h"
#include <SoftwareSerial.h>
#include "MainProcess.h"

extern "C" {
	#include <string.h>
	#include <stdlib.h>
}

#define Modbus_DEBUG 1

extern MainDataStruct maindata;
extern RuntimeStatus runtimedata;
extern HardwareSerial *cmd_port;

ModbusRTU_Protocol::ModbusRTU_Protocol(HardwareSerial* cmdport, const uint32_t baudrate = 115200)
{
#if Modbus_DEBUG	
	cmd_port->println("ModbusRTU_Protocol()." );
#endif
	modbus_port = cmdport;
	((HardwareSerial*)modbus_port)->begin(baudrate);
	
	cmdQueue = new Queue(sizeof(ModbusRTUCmdRec), 10, FIFO);
#if Modbus_DEBUG	
	cmd_port->println("ModbusRTU_Protocol Q Cnt: " + String(cmdQueue->getCount()));
#endif
	
	cmdRec.datalen = 0;
    DataBuffLen = 0;

	ProcessIndex = 0;
	ProcessTimeCnt = 0;
}

ModbusRTU_Protocol::ModbusRTU_Protocol(SoftwareSerial* cmdport, const uint32_t baudrate = 115200)
{
#if Modbus_DEBUG	
	cmd_port->println("ModbusRTU_Protocol()." );
#endif
	modbus_port = cmdport;
	((SoftwareSerial*)modbus_port)->begin(baudrate);
	SWSerial = true;
	
	cmdQueue = new Queue(sizeof(ModbusRTUCmdRec), 10, FIFO);
#if Modbus_DEBUG	
	cmd_port->println("ModbusRTU_Protocol Q Cnt: " + String(cmdQueue->getCount()));
#endif

	cmdRec.datalen = 0;
    DataBuffLen = 0;
	ProcessIndex = 0;
	ProcessTimeCnt = 0;
}

void ModbusRTU_Protocol::SendCommandQ(void)
{

	if((millis() - SendCmdTimeCnt) >= TIME_MBRTU_CMD_INTERVAL)
	{
		if(cmdQueue->isEmpty() == false)
		{
			if(!cmdQueue->pop(&cmdRec))
			{
				cmdRec.datalen = 0;
			}
		}
		if(cmdRec.datalen > 0)
		{
			if(modbus_port->availableForWrite())
			{
				modbus_port->write(cmdRec.data, cmdRec.datalen);
#if Modbus_DEBUG
			cmd_port->print("ModbusRTU_Protocol::SendCommandQ: ");
			for(int i=0; i<cmdRec.datalen; i++)
				cmd_port->print(String(cmdRec.data[i], HEX)+ " ");
			cmd_port->println();
#endif
			if(cmdRec.retrycnt < MBRTU_CMD_RETRY_MAX){
				cmdRec.retrycnt ++;
             }
			else
				cmdRec.datalen = 0;

			SendCmdTimeCnt = millis();
			}
		}
	}

}

void ModbusRTU_Protocol::Process(void)
{
	static uint8_t preProcessIndex = 0xff;
	int ret, i, inputCount=0;
	int retry=0;
	
#if Modbus_DEBUG
	if(preProcessIndex != ProcessIndex)
	{
		preProcessIndex = ProcessIndex;
		cmd_port->println("ProcessIndex: " + String(ProcessIndex, DEC));
	}
#endif
	switch(ProcessIndex)
	{
		case 0:
		{
			if(modbus_port->available())
			{
#if Modbus_DEBUG	
				cmd_port->println("modbus_port->available()");
#endif
				ProcessTimeCnt = millis();
				cmd_port->println("ProcessTimeCnt: " + String(ProcessTimeCnt));
				reclen = 0;
				ProcessIndex ++;
			}
			break;
		}
		case 1:
		{
			if((millis() - ProcessTimeCnt) > TIME_RECIVE_DATA_DELAY)
			{
#if Modbus_DEBUG	
				cmd_port->println("delay " + String(TIME_RECIVE_DATA_DELAY) +" ms");
#endif
				ProcessIndex ++;
			}
			break;
		}
		case 2:
		{
			while(modbus_port->available())
			{
				ret = modbus_port->read();
                DataBuff[DataBuffLen++] = (char)ret;
//				recdata[reclen++] = (char)ret;
			}
#if Modbus_DEBUG	
//			cmd_port->print("Modbus_Protocol recive (" + String(reclen) + String("): ") );
//			for(i=0; i<reclen; i++)
//				cmd_port->print(String(recdata[i], HEX)+ String(","));
			cmd_port->print("Modbus_Protocol recive (" + String(DataBuffLen) + String("): ") );
			for(i=0; i<DataBuffLen; i++)
				cmd_port->print(String(DataBuff[i], HEX)+ String(","));
			cmd_port->println();
#endif
	        ReciveTime = millis();
			ProcessIndex = 0;
			break;
		}
	}
    while(SplitRecvice()){
        CheckReciveData();
        cmd_port->println("SplitRecvice ING.");
    }
	SendCommandQ();
}


bool ModbusRTU_Protocol::SplitRecvice(void)
{
	int i, starti=-1, endi=-1;
	bool result = false;
	if(DataBuffLen >= 129)	
		DataBuffLen = 0;
	if(DataBuffLen >= MBRTU_CMD_LEN_BASE)
	{
		//for(i=0; i<DataBuffLen-2; i++)
		for(i=0; i<=DataBuffLen; i++)
			if((DataBuff[i] == 0x01))
			{
				if((DataBuff[i+1] == 0x04) || (DataBuff[i+1] == 0x84)) //Data Length
				{
					bool match = true;
//					if(((DataBuff[i+1] == 9) || (DataBuff[i+1] == 13))
//						&&(DataBuff[i+2] == 0x08))//HMI_CMD_DATA_INDICATION
//					{
//						if((DataBuff[i+4] == ControllerTagID) || (DataBuff[i+4] == ResponseTagID)
//							|| (DataBuff[i+5] == ControllerTagID) || (DataBuff[i+5] == ResponseTagID)
//							|| (DataBuff[i+6] == ControllerTagID) || (DataBuff[i+6] == ResponseTagID)
//							|| (DataBuff[i+7] == ControllerTagID) || (DataBuff[i+7] == ResponseTagID)
//							)
//							match = false;
//					}
					if(match)
					{
						starti = i;
						break;
					}
				}
			}
		if(starti > -1)
		{
			endi = DataBuff[starti + MBRTU_CMD_BYTE_LENGTH] + 5;
			if(DataBuffLen >= endi)
			{
#if Modbus_DEBUG	
					cmd_port->println("SplitRecvice Datlen: " + String(DataBuffLen) +", Starti: " + String(starti));
					cmd_port->println("Len: " + String(DataBuff[MBRTU_CMD_BYTE_LENGTH]) + ", Endi: " + String(endi));
#endif
				memcpy(recdata, &DataBuff[starti], DataBuff[MBRTU_CMD_BYTE_LENGTH]+5);
				reclen = DataBuff[MBRTU_CMD_BYTE_LENGTH]+5;
#if Modbus_DEBUG	
					cmd_port->println("SplitRecvice: ");
					for(i=0; i<reclen; i++)
						cmd_port->print(String(recdata[i], HEX) + " ");
					cmd_port->println();
#endif
				for(i=0; i<DataBuffLen-(reclen+starti); i++)
					DataBuff[i] = DataBuff[starti+i];
				DataBuffLen -= (reclen+starti);
				result = true;
#if Modbus_DEBUG	
					cmd_port->println("SplitRecvice result: " + String(result));
					cmd_port->print("HIM Data Buff (" + String(DataBuffLen) + String("): ") );
					for(i=0; i<DataBuffLen; i++)
						cmd_port->print(String(DataBuff[i], HEX)+ String(","));
					cmd_port->println();
#endif
			}
		}
	}

	if(DataBuffLen > 0)
		if(!result && ((millis() - ReciveTime) > TIME_RECIVE_DATA_OVERDUE))
		{
#if Modbus_DEBUG	
				cmd_port->println("millis: " + String(millis()) + ", ReciveTime: " + String(ReciveTime));
				cmd_port->println("Clear DataBuff(" + String(DataBuffLen) + ")");
#endif
			DataBuffLen = 0;
		}
	return result;
}


uint16_t ModbusRTU_Protocol::ComputeCRC(uint8_t *buf, uint8_t length)
{
	unsigned short crc = 0xFFFF;
	int i, j;
	unsigned char LSB;
	for (i = 0; i < length; i++) {
		crc ^= buf[i];
		for (j = 0; j < 8; j++) {
	  		LSB = crc & 1;
	  		crc = crc >> 1;
	  		if (LSB) {
	    		crc ^= 0xA001;
	  		}
		}
	}
	return ((crc & 0xFF00) >> 8) | ((crc & 0x0FF) << 8 );
}



bool ModbusRTU_Protocol::CheckReciveData()
{
	uint8_t rec_crc_Hi, rec_crc_Lo;
	uint8_t cmp_crc_Hi, cmp_crc_Lo;
	long value;
	uint16_t CheckCRC;
    runtimedata.WattageReadData = -1;
	if(recdata[0] != 0x01)
	{
#if 0//Modbus_DEBUG	
		cmd_port->println("Byte 0: " + String(recdata[0], HEX) + " != 0x01");
#endif
		return false;
	}
	rec_crc_Hi = recdata[reclen - 1];
	rec_crc_Lo = recdata[reclen - 2];
#if 0//Modbus_DEBUG
	cmd_port->println("rec_crc_Lo: " + String(rec_crc_Lo, HEX));
	cmd_port->println("rec_crc_Hi: " + String(rec_crc_Hi, HEX));
#endif
	CheckCRC = ComputeCRC(recdata, reclen-2);
	cmp_crc_Hi = CheckCRC & 0xff;
	cmp_crc_Lo = CheckCRC >> 8;
#if 0//Modbus_DEBUG
	cmd_port->println("cmp_crc_Lo: " + String(cmp_crc_Lo, HEX));
	cmd_port->println("cmp_crc_Hi: " + String(cmp_crc_Hi, HEX));
#endif
	
	if((cmp_crc_Hi != rec_crc_Hi))
	{
		
#if 0//Modbus_DEBUG
		cmd_port->println("CRC Hi Fail: " + String(rec_crc_Hi, HEX) + " != " +  String(cmp_crc_Hi, HEX));
#endif
		return false;
	}
	
	if((cmp_crc_Lo != rec_crc_Lo))
	{
#if 0//Modbus_DEBUG	
		cmd_port->println("CRC Lo Fail: " + String(rec_crc_Lo, HEX) + " != " +	String(cmp_crc_Lo, HEX));
#endif
		return false;
	}
    
	
    if(recdata[1] == 0x04)
    {
		if(recdata[2] == 0x04)
		{
			value = ((uint32_t) recdata[4])
					|((uint32_t) recdata[3]<<8)
					|((uint32_t) recdata[6]<<16)
					|((uint32_t) recdata[5]<<24);
			cmd_port->println("Wattage value: " + String(value));
            runtimedata.WattageReadData = value;
		}
    }
	if(recdata[1] == 0x03)
	{	//讀取暫存器
		if(recdata[2] == 0x02)
		{
			value = ((uint32_t) recdata[3]<<8)
					|((uint32_t) recdata[4]);
			cmd_port->println("value: " + String(value));
		}
		else if(recdata[2] == 0x04)
		{
			value = ((uint32_t) recdata[4])
					|((uint32_t) recdata[3]<<8)
					|((uint32_t) recdata[6]<<16)
					|((uint32_t) recdata[5]<<24);
			cmd_port->println("value: " + String(value));
		}
	}
	if(recdata[1] == 0x84){}//Error Msg
	cmdRec.datalen = 0;
	return true;
}

bool ModbusRTU_Protocol::SendWattageCommand()
{
	ModbusRTUCmdRec rec;
	int i;
    uint8_t StartAddr = 0x03;
    uint8_t EndAddr = 0x02;
	uint16_t CheckCRC;
    rec.datalen = 8;
    rec.data[MBRTU_CMD_BYTE_ID] = 0x01;
    rec.data[MBRTU_CMD_BYTE_FUNCTION] = 0x04;
    for(uint8_t i=0; i<2; i++)
        rec.data[MBRTU_CMD_BYTE_Addr + i] = (StartAddr >> (1-i)*8)& 0xff;
    for(uint8_t i=0; i<2; i++)
        rec.data[MBRTU_CMD_BYTE_Data + i] = (EndAddr >> (1-i)*8)& 0xff;
	CheckCRC = ComputeCRC(rec.data, rec.datalen-2);
	rec.data[rec.datalen-1] = CheckCRC & 0xff;
	rec.data[rec.datalen-2] = CheckCRC >> 8;	

	rec.retrycnt = 0;
//    runtimedata.ReadWattageRetryTimes = 0;
	cmdQueue->push(&rec);
	return true;
}
bool ModbusRTU_Protocol::Send_Command(uint8_t      ID,  uint8_t FunctionCode, uint32_t Addr, uint32_t Datacnt, uint32_t value)
{

	ModbusRTUCmdRec rec;
	int i;
	uint16_t CheckCRC;
	if(Datacnt > 0) {
		rec.datalen = 6+3+Datacnt*2;
		rec.data[MBRTU_CMD_BYTE_ID] = ID;
		rec.data[MBRTU_CMD_BYTE_FUNCTION] = FunctionCode;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Addr + i] = (Addr >> (1-i)*8)& 0xff;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Data + i] = (Datacnt >> (1-i)*8)& 0xff;
		rec.data[MBRTU_CMD_BYTE] = Datacnt*2;

		if(Datacnt > 1){
			rec.data[MBRTU_CMD_BYTE + 3] = ((value >> 24) & 0xFF);
			rec.data[MBRTU_CMD_BYTE + 4] = ((value >> 16) & 0xFF);
			rec.data[MBRTU_CMD_BYTE + 1] = ((value >> 8) & 0XFF);
			rec.data[MBRTU_CMD_BYTE + 2] = ((value & 0XFF));
		}
		else{
			for(uint8_t i=0; i<Datacnt*2; i++)
				rec.data[MBRTU_CMD_BYTE + 1 + i] = (value >> (Datacnt*2-1-i)*8)& 0xff;
		}
	}
	else if(Datacnt == 0) {
		rec.datalen = 6+2+Datacnt*2;		
		rec.data[MBRTU_CMD_BYTE_ID] = 0x01;
		rec.data[MBRTU_CMD_BYTE_FUNCTION] = 0x06;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Addr + i] = (Addr >> (1-i)*8)& 0xff;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Data + i] = (value >> (1-i)*8)& 0xff;
	}
	CheckCRC = ComputeCRC(rec.data, rec.datalen-2);
	rec.data[rec.datalen-1] = CheckCRC & 0xff;
	rec.data[rec.datalen-2] = CheckCRC >> 8;	

	rec.retrycnt = 0;
	cmdQueue->push(&rec);
	return true;
}

bool ModbusRTU_Protocol::Set_RPM(uint32_t value)
{
//	{0x01, 0x10, 0x01, 0x32, 0x0, 0x01, 0x02, 0x01, 0x90, 0x5A, 0xA5 };//位置模式速度
	ModbusRTUCmdRec rec;
	int i;
	uint16_t CheckCRC;
	uint32_t Addr = 306;
	uint32_t Datacnt = 1;
//	rec.datalen = 11;
	if(Datacnt > 0) rec.datalen = 6+3+Datacnt*2;
	else if(Datacnt == 0) rec.datalen = 6+2+Datacnt*2;
	rec.data[MBRTU_CMD_BYTE_ID] = 0x01;
	rec.data[MBRTU_CMD_BYTE_FUNCTION] = 0x10;
	for(uint8_t i=0; i<2; i++)
		rec.data[MBRTU_CMD_BYTE_Addr + i] = (Addr >> (1-i)*8)& 0xff;
	for(uint8_t i=0; i<2; i++)
		rec.data[MBRTU_CMD_BYTE_Data + i] = (Datacnt >> (1-i)*8)& 0xff;
	rec.data[MBRTU_CMD_BYTE] = Datacnt*2;
	for(uint8_t i=0; i<Datacnt*2; i++)
		rec.data[MBRTU_CMD_BYTE + 1 + i] = (value >> (Datacnt*2-1-i)*8)& 0xff;
	CheckCRC = ComputeCRC(rec.data, rec.datalen-2);
	rec.data[rec.datalen-1] = CheckCRC & 0xff;
	rec.data[rec.datalen-2] = CheckCRC >> 8;	
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
	return true;
}
bool ModbusRTU_Protocol::Set_Step(uint32_t value)
{
//	{0x01, 0x10, 0x01, 0x39, 0x0, 0x2, 0x04, 0x0, 0x0, 0x0, 0x0, 0x5A, 0xA5};//運行脈衝數

	ModbusRTUCmdRec rec;
	int i;
	uint16_t CheckCRC;
	uint32_t Addr = 313;
	uint32_t Datacnt = 2;
//	rec.datalen = 13;
	if(Datacnt > 0) rec.datalen = 6+3+Datacnt*2;
	else if(Datacnt == 0) rec.datalen = 6+2+Datacnt*2;
	rec.data[MBRTU_CMD_BYTE_ID] = 0x01;
	rec.data[MBRTU_CMD_BYTE_FUNCTION] = 0x10;
	for(uint8_t i=0; i<2; i++)
		rec.data[MBRTU_CMD_BYTE_Addr + i] = (Addr >> (1-i)*8)& 0xff;
	for(uint8_t i=0; i<2; i++)
		rec.data[MBRTU_CMD_BYTE_Data + i] = (Datacnt >> (1-i)*8)& 0xff;
	rec.data[MBRTU_CMD_BYTE] = Datacnt*2;
//	for(uint8_t i=0; i<Datacnt*2; i++)
//		rec.data[MBRTU_CMD_BYTE + 1 + i] = (value >> (Datacnt*2-1-i)*8)& 0xff;
	rec.data[MBRTU_CMD_BYTE + 3] = ((value >> 24) & 0xFF);
  	rec.data[MBRTU_CMD_BYTE + 4] = ((value >> 16) & 0xFF);
  	rec.data[MBRTU_CMD_BYTE + 1] = ((value >> 8) & 0XFF);
  	rec.data[MBRTU_CMD_BYTE + 2] = ((value & 0XFF));
	CheckCRC = ComputeCRC(rec.data, rec.datalen-2);
	rec.data[rec.datalen-1] = CheckCRC & 0xff;
	rec.data[rec.datalen-2] = CheckCRC >> 8;	
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
}
bool ModbusRTU_Protocol::Absolute_position()
{
//	{0x01, 0x06, 0x01, 0x43, 0x0, 0x01, 0x5A, 0xA5};//絕對位置
	ModbusRTUCmdRec rec;
	int i;
	uint16_t CheckCRC;
	uint32_t Addr = 323;
	uint32_t Datacnt = 0;
	uint32_t funcData = 1;
	if(Datacnt == 0) {
		rec.datalen = 6+2+Datacnt*2;		
		rec.data[MBRTU_CMD_BYTE_ID] = 0x01;
		rec.data[MBRTU_CMD_BYTE_FUNCTION] = 0x06;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Addr + i] = (Addr >> (1-i)*8)& 0xff;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Data + i] = (funcData >> (1-i)*8)& 0xff;
	}
	CheckCRC = ComputeCRC(rec.data, rec.datalen-2);
	rec.data[rec.datalen-1] = CheckCRC & 0xff;
	rec.data[rec.datalen-2] = CheckCRC >> 8;	
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
	return true;
}

bool ModbusRTU_Protocol::Emergency_Stop()
{
//	{0x01, 0x06, 0x01, 0x43, 0x0, 0x07, 0x5A, 0xA5};//緊急停止

	ModbusRTUCmdRec rec;
	int i;
	uint16_t CheckCRC;
	uint32_t Addr = 323;
	uint32_t Datacnt = 0;
	uint32_t funcData = 7;
	if(Datacnt == 0) {
		rec.datalen = 6+2+Datacnt*2;		
		rec.data[MBRTU_CMD_BYTE_ID] = 0x01;
		rec.data[MBRTU_CMD_BYTE_FUNCTION] = 0x06;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Addr + i] = (Addr >> (1-i)*8)& 0xff;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Data + i] = (funcData >> (1-i)*8)& 0xff;
	}
	CheckCRC = ComputeCRC(rec.data, rec.datalen-2);
	rec.data[rec.datalen-1] = CheckCRC & 0xff;
	rec.data[rec.datalen-2] = CheckCRC >> 8;	
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
	return true;
}

bool ModbusRTU_Protocol::Search_Home()
{
//	{0x01, 0x06, 0x01, 0x43, 0x0, 0xC, 0x5A, 0xA5};//回原點

	ModbusRTUCmdRec rec;
	int i;
	uint16_t CheckCRC;
	uint32_t Addr = 323;
	uint32_t Datacnt = 0;
	uint32_t funcData = 12;
	if(Datacnt == 0) {
		rec.datalen = 6+2+Datacnt*2;		
		rec.data[MBRTU_CMD_BYTE_ID] = 0x01;
		rec.data[MBRTU_CMD_BYTE_FUNCTION] = 0x06;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Addr + i] = (Addr >> (1-i)*8)& 0xff;
		for(uint8_t i=0; i<2; i++)
			rec.data[MBRTU_CMD_BYTE_Data + i] = (funcData >> (1-i)*8)& 0xff;
	}
	CheckCRC = ComputeCRC(rec.data, rec.datalen-2);
	rec.data[rec.datalen-1] = CheckCRC & 0xff;
	rec.data[rec.datalen-2] = CheckCRC >> 8;	
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
	return true;
}


