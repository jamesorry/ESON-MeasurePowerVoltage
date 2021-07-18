#include "UserCommand.h"
#include "EEPROM_Function.h"
#include "MainProcess.h"
#include "hmi.h"
#include "ModbusRTU_protocol.h"
#include "HMI_Command.h"
#include "Timer.h"

HardwareSerial *cmd_port;
HardwareSerial *rs485_port;
//test
extern MainDataStruct maindata;
extern RuntimeStatus runtimedata;

ModbusRTU_Protocol *modbuscmd;
HMI_Command *hmicmd;

void setup() {
	cmd_port = &CMD_PORT;
	cmd_port->begin(CMD_PORT_BR);
    
	modbuscmd = new ModbusRTU_Protocol(&RS485_PORT, RS485_PORT_BR);
    hmicmd = new HMI_Command(&HMI_PORT, HMI_PORT_BR);
    
	TimerInit(1, 10000);
	READ_EEPROM();
	MainProcess_Init();
	buzzerPlay(500);
}

void loop() {
    UserCommand_Task();
    modbuscmd->Process();
    hmicmd->Process();
    MainProcess_Task();
	if(runtimedata.UpdateEEPROM)
	{
		runtimedata.UpdateEEPROM = false;
		WRITE_EEPROM(); //maindata內的值都會寫到EEPROM
	}
}

uint16_t  msCnt = 0;
ISR(TIMER1_COMPA_vect)
{
    MainProcess_Timer();

	msCnt += TIMER_INTERVAL_MS;
	if(msCnt >= 1000) //1s
	{
		msCnt = 0;
	}
}

