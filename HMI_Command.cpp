#include "Arduino.h"
#include "hmi.h"
#include <SoftwareSerial.h>
#include "hmi_command.h"
#include "cppQueue.h"
#include "MainProcess.h"

extern "C" {
	#include <string.h>
	#include <stdlib.h>
}

#if 1
#define HMI_CMD_DEBUG		1
#define HMI_CMD_QUEUE_DEBUG	1
#endif
extern HardwareSerial *cmd_port;
extern MainDataStruct maindata;
extern RuntimeStatus runtimedata;
bool IsDataSend=false;
uint8_t resopnsebuf[32], resopnselen = 0;


HMI_Command::HMI_Command(HardwareSerial* port, const uint32_t baudrate, const uint8_t reqid, const uint8_t resid, const bool skipid)
{
	hmicmd_port = port;
	((HardwareSerial*)hmicmd_port)->begin(baudrate);

	ControllerTagID = reqid;
	ResponseTagID = resid;
	skipHMI_ID = skipid;

#if HMI_CMD_DEBUG	
	cmd_port->println("HMI_Command().");
#endif
	Update = false;
	reclen = 0;
	DataBuffLen = 0;

	cmdQueue = new Queue(sizeof(HMICmdRec), 10, FIFO);
#if HMI_CMD_DEBUG	
		cmd_port->println("init Q cnt: " + String(cmdQueue->getCount()));
#endif
	cmdRec.datalen = 0;

	ProcessIndex = 0;
	ProcessTimeCnt = 0;
	ParameterResult = 0;	//Success;
}

HMI_Command::HMI_Command(SoftwareSerial* port, const uint32_t baudrate, const uint8_t reqid, const uint8_t resid, const bool skipid)
{
	hmicmd_port = port;
	((SoftwareSerial*)hmicmd_port)->begin(baudrate);
	SWSerial = true;

	ControllerTagID = reqid;
	ResponseTagID = resid;
	skipHMI_ID = skipid;

#if HMI_CMD_DEBUG	
		cmd_port->println("HMI_Command().");
#endif
	Update = false;
	reclen = 0;
	DataBuffLen = 0;

	cmdQueue = new Queue(sizeof(HMICmdRec), 10, FIFO);
#if 0//HMI_CMD_DEBUG	
		cmd_port->println("init Q cnt: " + String(cmdQueue->getCount()));
#endif
	cmdRec.datalen = 0;

	ProcessIndex = 0;
	ProcessTimeCnt = 0;
	ParameterResult = 0;	//Success;
}

int qlen = 0xff;
void HMI_Command::SendCommandQ(void)
{
#if 0//HMI_CMD_QUEUE_DEBUG
	if(qlen != cmdQueue->getCount())
	{
		qlen = cmdQueue->getCount();
		cmd_port->println("Q Cnt:" + String(qlen, DEC) + " is empty: " + cmdQueue->isEmpty());
		if(qlen > 100)
			cmdQueue->clean();
	}
#endif

	if((millis() - SendCmdTimeCnt) >= TIME_HMI_CMD_INTERVAL)
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
			bool available = true;
			if(!SWSerial)
				available = ((HardwareSerial *)hmicmd_port)->availableForWrite();
			if(available)
			{
				hmicmd_port->write(cmdRec.data, cmdRec.datalen);
#if HMI_CMD_DEBUG
			cmd_port->println("cmdRec.datalen: " + String(cmdRec.datalen));
			cmd_port->print("HMI_Command::SendCommandQ: ");
			for(int i=0; i<cmdRec.datalen; i++)
				cmd_port->print(String(cmdRec.data[i], HEX)+ ",");
			cmd_port->println();
#endif
				if(cmdRec.datatype == QUEUE_DATA_TYPE_RESPONSE)
					cmdRec.datalen = 0;
				else
				{
					if(cmdRec.retrycnt < HMI_CMD_RETRY_MAX)
						cmdRec.retrycnt ++;
					else
						cmdRec.datalen = 0;
				}
				
				SendCmdTimeCnt = millis();
			}
		}
	}

}

bool HMI_Command::SplitRecvice(void)
{
	int i, starti=-1, endi=-1;
	bool result = false;
	if(DataBuffLen >= 129)	
		DataBuffLen = 0;
	if(DataBuffLen >= HMI_CMD_LEN_BASE)
	{
		//for(i=0; i<DataBuffLen-2; i++)
		for(i=0; i<=DataBuffLen-HMI_CMD_LEN_BASE; i++)
			if((DataBuff[i] == ControllerTagID) || (DataBuff[i] == ResponseTagID) || skipHMI_ID)
			{
				if(((DataBuff[i+1] < 32) && (DataBuff[i+1] >= HMI_CMD_LEN_BASE))
					&& (DataBuff[i+2] < 0x20)	//CMD ID < 0x20
					&& (DataBuff[i+3] < 0x50)	//HMI ID < 0x50
					)
				{
					bool match = true;
					if(((DataBuff[i+1] == 9) || (DataBuff[i+1] == 13))
						&&(DataBuff[i+2] == HMI_CMD_DATA_INDICATION))
					{
						if((DataBuff[i+4] == ControllerTagID) || (DataBuff[i+4] == ResponseTagID)
							|| (DataBuff[i+5] == ControllerTagID) || (DataBuff[i+5] == ResponseTagID)
							|| (DataBuff[i+6] == ControllerTagID) || (DataBuff[i+6] == ResponseTagID)
							|| (DataBuff[i+7] == ControllerTagID) || (DataBuff[i+7] == ResponseTagID)
							)
							match = false;
					}

					if(match)
					{
						starti = i;
						break;
					}
				}
			}
		if(starti > -1)
		{
			endi = DataBuff[HMI_CMD_BYTE_LENGTH+starti] + starti;
			if(DataBuffLen >= endi)
			{
#if HMI_CMD_DEBUG	
					cmd_port->println("SplitRecvice Datlen: " + String(DataBuffLen) +", Starti: " + String(starti));
					cmd_port->println("Len: " + String(DataBuff[HMI_CMD_BYTE_LENGTH]) + ", Endi: " + String(endi));
#endif
				memcpy(recdata, &DataBuff[starti], DataBuff[HMI_CMD_BYTE_LENGTH+starti]);
				reclen = DataBuff[HMI_CMD_BYTE_LENGTH+starti];
#if HMI_CMD_DEBUG	
					cmd_port->println("SplitRecvice: ");
					for(i=0; i<reclen; i++)
						cmd_port->print(String(recdata[i], HEX) + " ");
					cmd_port->println();
#endif
				for(i=0; i<DataBuffLen-(reclen+starti); i++)
					DataBuff[i] = DataBuff[starti+i];
				DataBuffLen -= (reclen+starti);
				result = true;
#if HMI_CMD_DEBUG	
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
#if HMI_CMD_DEBUG	
				cmd_port->println("millis: " + String(millis()) + ", ReciveTime: " + String(ReciveTime));
				cmd_port->println("Clear DataBuff(" + String(DataBuffLen) + ")");
#endif
			DataBuffLen = 0;
		}
	return result;
}

void HMI_Command::Process(void)
{
	static uint8_t preProcessIndex = 0xff;
	int ret,i,inputCount=0;
	int retry=0;
#if HMI_CMD_DEBUG
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
			if(hmicmd_port->available())
			{
#if HMI_CMD_DEBUG	
				cmd_port->println("hmicmd_port->available()");
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
#if HMI_CMD_DEBUG	
				cmd_port->println("delay " + String(TIME_RECIVE_DATA_DELAY) +" ms");
#endif
				ProcessIndex ++;
			}
			break;
		}
		case 2:
		{
			while(hmicmd_port->available() && (DataBuffLen < HMI_CMD_DATA_MAX_LEN))
			{
				ret = hmicmd_port->read();
//				DataBuff[DataBuffLen++] = (char)ret;
				recdata[reclen++] = (char)ret;
			}
			MsgUpdate = true;
//			Update = true;
#if HMI_CMD_DEBUG	
				cmd_port->print("HMI CMD recive (" + String(reclen) + String("): ") );
				for(i=0; i<reclen; i++)
					cmd_port->print(String(recdata[i], HEX)+ String(","));
				cmd_port->println();
#endif

			Receive_HMI_CMD = CheckReciveData();
			ReciveTime = millis();
			ProcessIndex = 0;
			break;
		}
	}
//	while(SplitRecvice())
//		CheckReciveData();
	SendCommandQ();
}

void HMI_Command::ReciveDataTest(uint8_t *data, uint8_t datalen)
{
	memcpy(&DataBuff[DataBuffLen], data, datalen);
	DataBuffLen += datalen;
#if HMI_CMD_DEBUG	
		cmd_port->print("ReciveDataTest (" + String(DataBuffLen) + String("): ") );
		for(int i=0; i<DataBuffLen; i++)
			cmd_port->print(String(DataBuff[i], HEX)+ String(","));
		cmd_port->println();
#endif
	ReciveTime = millis();
#if HMI_CMD_DEBUG
		cmd_port->println("ReciveDataTest: " + String(ReciveTime));
#endif
	//while(SplitRecvice())
	//	CheckReciveData();
	
}


uint8_t HMI_CMD_ComputeCRC(uint8_t *buff)
{
	uint8_t cmp_crc = 0x00, i;
	for(i=0; i<buff[HMI_CMD_BYTE_LENGTH]-1; i++)
		cmp_crc -= buff[i];
	return (uint8_t)(cmp_crc & 0xFF);
}

bool HMI_Command::Response_Ping()
{
	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_RESPONSE;
	rec.data[HMI_CMD_BYTE_TAGID] = ResponseTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_PING;
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);

#if HMI_CMD_DEBUG
	cmd_port->println("Response_Ping()");
#endif
	return true;
}

void HMI_Command::Request_Ping()
{
	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_PING;
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);

#if HMI_CMD_DEBUG
	cmd_port->println("Request_Ping()");
#endif
	return true;
}

#if 0
void HMI_Command::Request_Record_Mode_Result(uint32_t OK_Num, uint32_t NG_Num, uint32_t Process_Time, uint32_t Pulse_Num)	//0x01
{
//	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = 16;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_RECORD_MODE_RESELT;
	for(uint8_t i=0; i<4; i++)
		rec.data[HMI_CMD_BYTE_DATA+i] = (OK_Num >> (3-i)*8)& 0xff;
	for(uint8_t i=4; i<8; i++)
		rec.data[HMI_CMD_BYTE_DATA+i] = (NG_Num >> (7-i)*8)& 0xff;
	for(uint8_t i=8; i<10; i++)
		rec.data[HMI_CMD_BYTE_DATA+i] = (Process_Time >> (9-i)*8)& 0xff;
//	2021/02/04 更改

	for(uint8_t i=10; i<12; i++)
		rec.data[HMI_CMD_BYTE_DATA+i] = (Pulse_Num >> (11-i)*8)& 0xff;	
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);

#if HMI_CMD_DEBUG
	Serial.println("rec.datalen: " + String(rec.datalen));
	cmd_port->println("Request_Record_Mode_Result()");
#endif
	return true;
}


void HMI_Command::Request_Mode_select(uint8_t Data)				//0x02
{
	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE + 1;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_MODE_SELECT;
	rec.data[HMI_CMD_BYTE_DATA] = Data; //0 = Normal mode, 1 = Record mode
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);

#if HMI_CMD_DEBUG
	cmd_port->println("Request_Mode_select()");
#endif
	return true;
}

void HMI_Command::Request_Start_Time_Parameter(uint8_t CaseType) //0x03			//誠盈0x03
{
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
//	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE + 1;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_START_TIME_PARAMETER;
	switch (CaseType)
	{
		case CASE_FRICTION: //磨擦
			
			break;
		case CASE_PULSE: 	//旋鈕
			long pulse_averageValue = (int)(runtimedata.pulse_averageValue*1000);
			cmd_port->println("pulse_averageValue: " + String(pulse_averageValue));
			long Pulse_high= (runtimedata.SetPulse_high.toFloat()*100);
			cmd_port->println("Pulse_high: " + String(Pulse_high));
			long Pulse_low= (runtimedata.SetPulse_low.toFloat()*100);
			cmd_port->println("Pulse_low: " + String(Pulse_low));
			long Period_Time = 120;
			long Peak2Peak = 3300;
			rec.data[HMI_CMD_BYTE_LENGTH] = 4 + 20;
#if HMI_CMD_DEBUG
			cmd_port->println("CaseType: " + String(CaseType));
			cmd_port->println("rec.data[HMI_CMD_BYTE_LENGTH]: " + String(rec.data[HMI_CMD_BYTE_LENGTH]));
#endif		
			for(uint8_t i=0; i<2; i++)	//馬達轉速
				rec.data[HMI_CMD_BYTE_DATA+i] = (maindata.rpmValue >> (1-i)*8)& 0xff;
			for(uint8_t i=2; i<6; i++)	//A點
				rec.data[HMI_CMD_BYTE_DATA+i] = (maindata.a_pointValue >> (5-i)*8)& 0xff;
			for(uint8_t i=6; i<10; i++)	//B點
				rec.data[HMI_CMD_BYTE_DATA+i] = (maindata.b_pointValue >> (9-i)*8)& 0xff;
			for(uint8_t i=10; i<12; i++)	//1pulse所需平均時間
				rec.data[HMI_CMD_BYTE_DATA+i] = ((pulse_averageValue) >> (11-i)*8)& 0xff;
			for(uint8_t i=12; i<14; i++)	//1pulse上限判定標準時間
				rec.data[HMI_CMD_BYTE_DATA+i] = (( Pulse_high) >> (13-i)*8)& 0xff;
			for(uint8_t i=14; i<16; i++)	//1pulse下限判定標準時間
				rec.data[HMI_CMD_BYTE_DATA+i] = ((Pulse_low) >> (15-i)*8)& 0xff;
			for(uint8_t i=16; i<18; i++)	//Period Time(ms)(2 bytes)
				rec.data[HMI_CMD_BYTE_DATA+i] = ((Period_Time) >> (17-i)*8)& 0xff;
			for(uint8_t i=18; i<20; i++)	//Peak2Peak(mV)(2 bytes)
				rec.data[HMI_CMD_BYTE_DATA+i] = ((Peak2Peak) >> (19-i)*8)& 0xff;
			break;
		case CASE_BUTTUN:	//按鈕
			
			break;
		case CASE_SAFETY_SWITCH:	//安全開關
		
			break;
	}
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);

#if HMI_CMD_DEBUG
	cmd_port->println("Request_Mode_select()");
#endif
	return true;
}

void HMI_Command::Request_Normal_Mode_End_Result(uint32_t total_times, uint32_t OK_Num, uint32_t NG_Num)				//0x04
{
	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE + 12;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_NORMAL_MODE_END_RESULT;
	for(uint8_t i=0; i<4; i++)
		rec.data[HMI_CMD_BYTE_DATA+i] = (total_times >> (3-i)*8)& 0xff;
	for(uint8_t i=4; i<8; i++)
		rec.data[HMI_CMD_BYTE_DATA+i] = (OK_Num >> (7-i)*8)& 0xff;
	for(uint8_t i=8; i<12; i++)
		rec.data[HMI_CMD_BYTE_DATA+i] = (NG_Num >> (11-i)*8)& 0xff;
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
#if HMI_CMD_DEBUG
	cmd_port->println("Request_Normal_Mode_End_Result()");
#endif
	return true;
}

void HMI_Command::Request_Launch_Stop(uint8_t Data)				//0x05
{
	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE + 1;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_LAUNCH_STOP;
	rec.data[HMI_CMD_BYTE_DATA] = Data;
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
#if HMI_CMD_DEBUG
	cmd_port->println("Request_Set_Test_Type()");
#endif
	return true;
}

void HMI_Command::Request_Average_Time_Volt()				//0x06
{
	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_Average_TIME_VOLT;
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
#if HMI_CMD_DEBUG
	cmd_port->println("Request_Set_Test_Type()");
#endif
	return true;
}


void HMI_Command::Request_Set_Test_Type(uint8_t type)				//0x07
{
	uint8_t i;
	HMICmdRec rec;
	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
	rec.data[HMI_CMD_BYTE_TAGID] = ControllerTagID;
	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE + 1;
	rec.data[HMI_CMD_BYTE_CMDID] = HMI_CMD_Set_Test_Type;
	rec.data[HMI_CMD_BYTE_DATA] = type;
	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
	rec.retrycnt = 0;
	cmdQueue->push(&rec);
#if HMI_CMD_DEBUG
	cmd_port->println("Request_Set_Test_Type()");
#endif
	return true;
}
#endif

uint8_t HMI_Command::CheckReciveData()
{
	uint8_t rec_crc, cmp_crc = 0x00, i;
	bool issupportcmd = false;
	if(reclen < HMI_CMD_LEN_BASE)
	{
#if 0//HMI_CMD_DEBUG	
		cmd_port->println("CheckReciveData reclen: " + String(reclen) + " Exit.");
#endif
		return -1;
	}
#if HMI_CMD_DEBUG	
		cmd_port->println("CheckReciveData reclen: " + String(reclen));
#endif
		//&& (recdata[HMI_CMD_BYTE_HMIID] == maindata.HMI_ID)
	if(reclen == recdata[HMI_CMD_BYTE_LENGTH])
	{
#if HMI_CMD_DEBUG	
		cmd_port->println("CheckReciveData reclen: " + String(reclen));
#endif

		rec_crc = recdata[recdata[HMI_CMD_BYTE_LENGTH] - 1];
		//for(i=0; i<reclen-1; i++)
		//	cmp_crc -= recdata[i];
		cmp_crc = HMI_CMD_ComputeCRC(recdata);
		
		if(cmp_crc != rec_crc)
		{
#if HMI_CMD_DEBUG	
			cmd_port->println("CRC Fail " + String(cmp_crc, HEX));
#endif
			return -1;
		}

#if HMI_CMD_DEBUG	
	cmd_port->println("cmdRec.datatype: " + String(cmdRec.datatype, HEX) + ", cmd: " + String(recdata[HMI_CMD_BYTE_CMDID], HEX));
#endif
//		if((cmdRec.datalen > 0)
//			&& (cmdRec.datatype == QUEUE_DATA_TYPE_INDICATION)
//			&& (cmdRec.data[HMI_CMD_BYTE_CMDID] != recdata[HMI_CMD_BYTE_CMDID])
//			)
//		{
//			if(cmdRec.retrycnt < HMI_CMD_RETRY_MAX)
//			{
//				cmdRec.last_ms = millis();
//				cmdQueue->push(&cmdRec);
//			}
//			cmdRec.datalen = 0;
//		}
//		else
		{
			if(cmdRec.datalen > 0)
			{
				if(cmdRec.retrycnt < HMI_CMD_RETRY_MAX)
				{
					cmdRec.last_ms = millis();
					cmdQueue->push(&cmdRec);
				}
				cmdRec.datalen = 0;
			}
#if HMI_CMD_DEBUG	
//	cmd_port->println("Response cmd: " + String(recdata[HMI_CMD_BYTE_CMDID], HEX));
	cmd_port->println("HMI_CMD: " + String(recdata[HMI_CMD_BYTE_CMDID], HEX));
#endif
			
			resopnsebuf[HMI_CMD_BYTE_TAGID] = ResponseTagID;
			
			switch(recdata[HMI_CMD_BYTE_CMDID])
			{
				case HMI_CMD_PING:
					cmd_port->println("HMI_CMD_PING.");
					issupportcmd = true;
					return 0;
					break;
				case HMI_CMD_RECORD_MODE_RESELT:
					cmd_port->println("HMI_CMD_RECORD_MODE_RESELT.");
					issupportcmd = true;
					return 1;
					break;
				case HMI_CMD_MODE_SELECT:
					cmd_port->println("HMI_CMD_MODE_SELECT.");
					issupportcmd = true;
					return 2;
					break;
				case HMI_CMD_START_TIME_PARAMETER:
					cmd_port->println("HMI_CMD_START_TIME_PARAMETER.");
					issupportcmd = true;
					return 3;
					break;
				case HMI_CMD_NORMAL_MODE_END_RESULT:
					cmd_port->println("HMI_CMD_NORMAL_MODE_END_RESULT.");
					issupportcmd = true;
					return 4;
					break;
				case HMI_CMD_LAUNCH_STOP:
					cmd_port->println("HMI_CMD_LAUNCH_STOP.");
					issupportcmd = true;
					return 5;
					break;
				case HMI_CMD_Average_TIME_VOLT:
					cmd_port->println("HMI_CMD_Average_TIME_VOLT.");
					Receive_PeriodTime = recdata[HMI_CMD_BYTE_CMDID+1]<<8 | recdata[HMI_CMD_BYTE_CMDID+2];
					Receive_Peak2Peak = recdata[HMI_CMD_BYTE_CMDID+3]<<8 | recdata[HMI_CMD_BYTE_CMDID+4];
					cmd_port->println("Receive_PeriodTime: " + String(Receive_PeriodTime));
					cmd_port->println("Receive_Peak2Peak: " + String(Receive_Peak2Peak));
					issupportcmd = true;
					return 6;
					break;
				case HMI_CMD_Set_Test_Type:
					cmd_port->println("HMI_CMD_Set_Test_Type.");
					issupportcmd = true;
					return 7;
					break;
			}
			if(issupportcmd)
			{
				cmdRec.datalen = 0;
			}	
		}
	}
	else
	{
#if HMI_CMD_DEBUG
		if(reclen != recdata[HMI_CMD_BYTE_LENGTH])
			cmd_port->println("Leng fail (" + String(reclen, DEC) + " != " + String(recdata[HMI_CMD_BYTE_LENGTH]) + String(")"));
#endif
		return -1;
	}
	
}




//FD 0B 01 00 00 00 C0 A8 03 65 27 00
//FD

bool HMI_Command::Send_Command(uint8_t cmdid, uint8_t tagid, uint8_t hmiid)
{

//	HMICmdRec rec;
//	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
//	rec.data[HMI_CMD_BYTE_TAGID] = tagid;
//	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE;
//	rec.data[HMI_CMD_BYTE_CMDID] = cmdid;
//	rec.data[HMI_CMD_BYTE_HMIID] = hmiid;
//	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
//	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
//	rec.retrycnt = 0;
//	cmdQueue->push(&rec);
//	
//	return true;
}

bool HMI_Command::Send_Command(uint8_t cmdid, uint8_t tagid, uint8_t hmiid, uint8_t data)
{

//	HMICmdRec rec;
//	rec.datatype = QUEUE_DATA_TYPE_INDICATION;
//	rec.data[HMI_CMD_BYTE_TAGID] = tagid;
//	rec.data[HMI_CMD_BYTE_LENGTH] = HMI_CMD_LEN_BASE + 1;
//	rec.data[HMI_CMD_BYTE_CMDID] = cmdid;
//	rec.data[HMI_CMD_BYTE_HMIID] = hmiid;
//	rec.data[HMI_CMD_BYTE_DATA] = data;
//	rec.data[rec.data[HMI_CMD_BYTE_LENGTH]-1] = HMI_CMD_ComputeCRC(rec.data);
//	rec.datalen = rec.data[HMI_CMD_BYTE_LENGTH];
//	rec.retrycnt = 0;
//	cmdQueue->push(&rec);
//	
//	return true;
}


