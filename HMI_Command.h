#ifndef _HMI_CMD_BT_COMMAND_H_
#define _HMI_CMD_BT_COMMAND_H_

#include "Arduino.h"
#include "cppQueue.h"

#define HMI_CMD_DATA_MAX_LEN 128+1

#define LCD_SHOW_DEBUG_MSG	0
#define LCD_SHOW_DEBUG_MSG_LINE	7

#define TIME_RECIVE_DATA_DELAY		0
#define TIME_RECIVE_DATA_OVERDUE	1000

typedef struct {
	uint8_t datatype;
	uint8_t retrycnt;
	uint8_t datalen;
	uint8_t data[40];

	uint32_t last_ms;
} HMICmdRec;



class HMI_Command
{
private:
	Queue	*cmdQueue;		//!< Queue implementation: FIFO LIFO
	HMICmdRec cmdRec;

	uint8_t	ProcessIndex;
	uint16_t ProcessTimeCnt;
	
	Stream *hmicmd_port = NULL;
	//HardwareSerial *swhmicmd_port = NULL;

	bool SWSerial = false;
	bool Update;

	uint8_t SendIndication;
	uint16_t IndicationDataType;

	bool Connected;

	uint8_t ParameterResult;
	uint8_t NeedSendRFIDData;
	uint8_t NeedSendBarcodeData;

	uint32_t SendCmdTimeCnt = 0;

	uint32_t	ReciveTime;

	bool skipHMI_ID;
	uint8_t recdata[HMI_CMD_DATA_MAX_LEN];
	uint8_t reclen;
	uint8_t DataBuff[HMI_CMD_DATA_MAX_LEN];
	uint8_t DataBuffLen;

public:
	
		uint8_t 	ControllerTagID;
		uint8_t 	ResponseTagID;
		uint8_t 	Receive_HMI_CMD;
		bool 		MsgUpdate;
		uint16_t	Receive_PeriodTime;
		uint16_t	Receive_Peak2Peak;
	HMI_Command(HardwareSerial* cmdport, const uint32_t baudrate=115200, const uint8_t reqid=0xFC, const uint8_t resid=0xFD, const bool skipid=false);
	HMI_Command(SoftwareSerial* cmdport, const uint32_t baudrate=115200, const uint8_t reqid=0xFC, const uint8_t resid=0xFD, const bool skipid=false);
	~HMI_Command();
	
	void Process(void);
	void Recive_Message();
	void Response_Message();
	void Response_CRC_Fail(uint8_t crc);

	bool Response_Ping();
//	誠盈專用
	void Request_Ping();
	void Request_Set_Test_Type(uint8_t type);
	void Request_Record_Mode_Result(uint32_t OK_Num, uint32_t NG_Num, uint32_t Process_Time, uint32_t Pulse_Num=0);
	void Request_Start_Time_Parameter(uint8_t CaseType);
	void Request_Normal_Mode_End_Result(uint32_t total_times, uint32_t OK_Num, uint32_t NG_Num);
	void Request_Launch_Stop(uint8_t Data);
	void Request_Average_Time_Volt();
	void Request_Mode_select(uint8_t Data=0);


	

	bool Send_Command(uint8_t cmdid, uint8_t tagid, uint8_t hmiid);
	bool Send_Command(uint8_t cmdid, uint8_t tagid, uint8_t hmiid, uint8_t data);

	bool SplitRecvice();
	uint8_t CheckReciveData();
	void SendCommandQ();
	void Set_HMIID(uint8_t id);
	void ReciveDataTest(uint8_t *data, uint8_t datalen);
	
	void CommandTest();
};

#define HMI_CMD_LEN_BASE			4

#define QUEUE_DATA_TYPE_RESPONSE	0
#define QUEUE_DATA_TYPE_INDICATION	1

#define HMI_CMD_RETRY_MAX	0	//10


//誠盈案子專用---------------------------------------
#define CASE_FRICTION							0
#define CASE_PULSE								1
#define CASE_BUTTUN								2
#define CASE_SAFETY_SWITCH						3

#define HMI_CMD_PING							0x00
#define HMI_CMD_RECORD_MODE_RESELT				0x01
#define HMI_CMD_MODE_SELECT						0x02
#define HMI_CMD_START_TIME_PARAMETER			0x03
#define HMI_CMD_NORMAL_MODE_END_RESULT			0X04
#define HMI_CMD_LAUNCH_STOP						0X05
#define HMI_CMD_Average_TIME_VOLT				0X06
#define HMI_CMD_Set_Test_Type					0x07
//---------------------------------------------------


#define HMI_CMD_PING							0x00
#define HMI_CMD_PARAMETER_SETTING				0x01
#define HMI_CMD_IO_STATUS						0x02
#define HMI_CMD_GO_TO_SERVO_POSITION			0x03
#define HMI_CMD_SEARCH_SENSOR					0x04

#define HMI_CMD_SET_HMI_ID						0x06
#define HMI_CMD_RUN								0x07
#define HMI_CMD_DATA_INDICATION					0x08
#define HMI_CMD_RESETTING						0x09
#define HMI_CMD_SET_SERVO_ON_OFF				0x0A
#define HMI_CMD_SAVE_SERVO_CURRENT_POSITION		0x0B
#define HMI_CMD_SET_SERVO_STEPS_MOVE			0x0C

#define HMI_CMD_SET_FLATCAR_QUERY_STATION		0x0D
#define HMI_CMD_SET_DO_STATE					0x0E
#define HMI_CMD_SET_FLATCAR_GRIP_STATE			0x0F
#define HMI_CMD_SET_FLATCAR_GO_TO_STATION		0x10
#define HMI_CMD_UPDATE_LINE_STATION_INFO		0x11

#define HMI_CMD_UP_LOW_MOVE						0x12		//需要再討論必要性
#define HMI_CMD_QUERY_SLAVE_HMI_DATA			0x13		

#define HMI_CMD_SET_MOTOR_SPEED_TYPE			0x16
//#define HMI_CMD_DEBUG							0xFE


#define HMI_CMD_BYTE_TAGID				0
#define HMI_CMD_BYTE_LENGTH				1
#define HMI_CMD_BYTE_CMDID				2
#define HMI_CMD_BYTE_HMIID				3
#define HMI_CMD_BYTE_DATA				3

#define HMI_CMD_DATA_INDICATION_RFID				0
#define HMI_CMD_DATA_INDICATION_HMI_ERROR			1
#define HMI_CMD_DATA_INDICATION_MOTOR_STATUS		2
#define HMI_CMD_DATA_INDICATION_LIFTING_MSG			3
#define HMI_CMD_DATA_INDICATION_PING				4
//#define HMI_CMD_DATA_INDICATION_FLATCAR_DATA		5
//#define HMI_CMD_DATA_INDICATION_LIFTING_DATA		6
#define HMI_CMD_DATA_INDICATION_FLATCAR_MSG			7
#define HMI_CMD_DATA_INDICATION_BROADCAST_ENABLE	8

#define HMI_CMD_TIME_RESEND_TIMEOUT		10000
#define TIME_HMI_CMD_INTERVAL			400

#define ERROR_CODE_NONE					0x0000
#define ERROR_CODE_EMO					0xE001
#define ERROR_CODE_RFID					0xE002

#define HMI_ID_MASTER_CONTROLLER		0x00
#define HMI_ID_FLATCAR					0x10		//20200924改成從1開始	
#define HMI_ID_LIFTING					0x21
#define HMI_ID_CONTROL_SIDE				0x31

uint8_t HMI_CMD_ComputeCRC(uint8_t *buff);

#endif	//_HMI_CMD_BT_COMMAND_H_
