#include "UserCommand.h"
#include "EEPROM_Function.h"
#include "MainProcess.h"
#include "hmi.h"
#include "ModbusRTU_protocol.h"
#include "HMI_Command.h"

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
	rs485_port = &RS485_PORT;
	rs485_port->begin(RS485_PORT_BR);

	modbuscmd = new ModbusRTU_Protocol(&RS485_PORT, RS485_PORT_BR);
    hmicmd = new HMI_Command(&BT_PORT, BT_PORT_BR);
    
	READ_EEPROM();
	MainProcess_Init();
	buzzerPlay(500);
}

void loop() {
    UserCommand_Task();
    modbuscmd->Process();
    hmicmd->Process();
    
	if(runtimedata.UpdateEEPROM)
	{
		runtimedata.UpdateEEPROM = false;
		WRITE_EEPROM(); //maindata內的值都會寫到EEPROM
		delay(1000);
	}
}
