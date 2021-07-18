#include "MainProcess.h"
#include <Adafruit_MCP23017.h>
#include "hmi.h"
#include "ModbusRTU_protocol.h"
#include "Timer.h"
#include "HMI_Command.h"

extern HardwareSerial *cmd_port;
MainDataStruct maindata;
RuntimeStatus runtimedata;
DigitalIO digitalio;
extern ModbusRTU_Protocol *modbuscmd;
extern HMI_Command *hmicmd;
Adafruit_MCP23017 extio[EXTIO_NUM];

void MainProcess_ReCheckEEPROMValue()
{
	if((maindata.HMI_ID == 0) || (maindata.HMI_ID > 128))
	{
		maindata.HMI_ID = 0;
		maindata.TestMaindataValue = 1000;
		runtimedata.UpdateEEPROM = true;
	}
}

void MainProcess_Init()
{
	int i,j;
	runtimedata.UpdateEEPROM = false;
	MainProcess_ReCheckEEPROMValue();

	for(i=0;i<INPUT_8_NUMBER+EXTIO_NUM;i++)
		digitalio.Input[i] = 0;
	
	for(i=0;i<OUTPUT_8_NUMBER+EXTIO_NUM;i++)
	{
		if(OUTPUT_NONE_ACTIVE == 0)
			digitalio.Output[i]	= 0;
		else
			digitalio.Output[i]	= 0xFF;
	}
		
	for(i=0; i<INPUT_8_NUMBER*8; i++)
	{
		pinMode(InputPin[i], INPUT);
	}
	for(i=0; i<OUTPUT_8_NUMBER*8; i++)
	{
		pinMode(OutputPin[i], OUTPUT);	
	}
	
	for(j=0; j<EXTIO_NUM; j++)
	{
		extio[j].begin(j);	  	// Default device address 0x20+j

		for(i=0; i<8; i++)
		{
			extio[j].pinMode(i, OUTPUT);  // Toggle LED 1
			extio[j].digitalWrite(i, OUTPUT_NONE_ACTIVE);
		}
	}
	for(i=0; i<OUTPUT_8_NUMBER*8; i++)
		digitalWrite(OutputPin[i], OUTPUT_NONE_ACTIVE);

	for(j=0; j<EXTIO_NUM; j++)
		for(i=0; i<8; i++)
		{
			extio[j].pinMode(i+8,INPUT);	 // Button i/p to GND
			extio[j].pullUp(i+8,HIGH);	 // Puled high to ~100k
		}
    runtimedata.Workindex[STATUS] = STATUS_STOP;
        
}

void ReadDigitalInput()
{
	uint8_t i,bi, j, value;
	String outstr = "";
	bool inputupdate = false;
	uint8_t input8 = 1;
	
	for(i=0; i<8; i++)
	{
		runtimedata.sensor[i] = digitalRead(InputPin[i]);
		if(runtimedata.sensor[i])
			{setbit(digitalio.Input[0], i);	}
		else
			{clrbit(digitalio.Input[0], i);	}
	}

	if(INPUT_8_NUMBER == 2)
	{
		for(i=0; i<8; i++)
		{
			runtimedata.sensor[i+8] = digitalRead(InputPin[i+8]);
			
			if(runtimedata.sensor[i+8])
				{setbit(digitalio.Input[1], i); }
			else
				{clrbit(digitalio.Input[1], i); }
		}
		input8 += 1;
	}

	if(EXTIO_NUM > 0)
	{
		for(i=0; i<8; i++)
		{
			runtimedata.sensor[i+8] = extio[0].digitalRead(i+8);
				
			if(runtimedata.sensor[i+input8*8])
				{setbit(digitalio.Input[input8], i);	}
			else
				{clrbit(digitalio.Input[input8], i);	}
		}
		input8 += 1;
	}
	if(EXTIO_NUM > 1)
	{
		for(i=0; i<8; i++)
		{
			runtimedata.sensor[i+input8*8] = extio[1].digitalRead(i+8);
			if(runtimedata.sensor[i+input8*8])
				{setbit(digitalio.Input[input8], i);	}
			else
				{clrbit(digitalio.Input[input8], i);	}
		}
	}

}

void WriteDigitalOutput()
{
	uint8_t i,bi, j, value;

	for(i=0; i<OUTPUT_8_NUMBER+EXTIO_NUM; i++)
	{
		if(digitalio.PreOutput[i] != digitalio.Output[i])
		{
			digitalio.PreOutput[i] = digitalio.Output[i];
			
			switch(i)
			{
				case 0: //onboard
					for(bi=0; bi<8; bi++)
					{
						value = getbit(digitalio.Output[i], bi);
						digitalWrite(OutputPin[bi], value);
					}
					break;

				case 1: //extern board 0
					for(bi=0; bi<8; bi++)
					{
						value = getbit(digitalio.Output[i], bi);
						if(OUTPUT_8_NUMBER == 2)
							digitalWrite(OutputPin[bi+8], value);
						else
							extio[0].digitalWrite(bi, value);
					}
					break;
				case 2: //extern board 1
					for(bi=0; bi<8; bi++)
					{
						value = getbit(digitalio.Output[i], bi);
						if(OUTPUT_8_NUMBER == 2)
							extio[0].digitalWrite(bi, value);
						else
							extio[1].digitalWrite(bi, value);
					}
					break;
				case 3: //extern board 1
					for(bi=0; bi<8; bi++)
					{
						value = getbit(digitalio.Output[i], bi);
						extio[1].digitalWrite(bi, value);
					}
					break;
			}	
		}
	}
}

void setOutput(uint8_t index, uint8_t hl)
{
	if(index < (OUTPUT_8_NUMBER*8))
	{
		digitalWrite(OutputPin[index], hl);
	}
	else
	{
		uint8_t extindex = index-(OUTPUT_8_NUMBER*8);
		uint8_t exi = extindex >> 3;
		uint8_t bi = extindex & 0x07;
		extio[exi].digitalWrite(bi, hl);
	}
	digitalio.Output[index] = hl;
}

uint8_t getInput(uint8_t index)
{
	uint8_t hl;
	if(index < (INPUT_8_NUMBER*8))
	{
		hl = digitalRead(InputPin[index]);
	}
	else
	{
		uint8_t extindex = index-(INPUT_8_NUMBER*8);
		uint8_t exi = extindex >> 3;
		uint8_t bi = extindex & 0x07;
		hl = extio[exi].digitalRead(bi+8);
	}

	digitalio.Input[index] = hl;
	return hl;
}

uint16_t	ReadVoltageTimer = 0;
uint16_t	CommunicationTimer = 0;
uint16_t	WaitTimer = 0;
void MainProcess_Timer()
{
    if(CommunicationTimer < 0xFFFF)
        CommunicationTimer += TIMER_INTERVAL_MS;
	if(ReadVoltageTimer < 0xFF00)
		ReadVoltageTimer += TIMER_INTERVAL_MS;
	if(WaitTimer < 0xFF00)
		WaitTimer += TIMER_INTERVAL_MS;
}


void MainProcess_Task()  // This is a task.
{
//	WriteDigitalOutput();
//	ReadDigitalInput();
    ProcessStasus();
}

#define HMI_CMD_WAIT_TIME 20000
void ProcessStasus()
{
    if(runtimedata.preWorkindex[STATUS] != runtimedata.Workindex[STATUS])
    {
        cmd_port->println("runtimedata.Workindex[STATUS]: " + String(runtimedata.Workindex[STATUS]));
        runtimedata.preWorkindex[STATUS] = runtimedata.Workindex[STATUS];
    }
    switch(runtimedata.Workindex[STATUS])
    {
        case STATUS_STOP:       //Ping PC, check pc connect or not
            if(CheckPing()){
                WaitTimer = 0;
                if(!getInput(InStartBtn))
                    runtimedata.Workindex[STATUS] = STATUS_CHECK_BTN;
            }
            break;
        case STATUS_CHECK_BTN:  //wait Start Btn push
            if(CheckBtn()){
                WaitTimer = 0;
                runtimedata.Workindex[STATUS] = STATUS_CHECK_START;
            }
            if(WaitTimer > 3000)
                runtimedata.Workindex[STATUS] = STATUS_STOP;
            break;
        case STATUS_CHECK_START: //check start
            if(CheckStartEnd(1)){
                ReadVoltageTimer = 0;
                WaitTimer = 0;
                runtimedata.Workindex[STATUS] = STATUS_READ_VOLTAGE;
            }
            if(WaitTimer > HMI_CMD_WAIT_TIME)
                runtimedata.Workindex[STATUS] = STATUS_STOP;
            break;
        case STATUS_READ_VOLTAGE:   //開始後1秒讀取
            if(ReadVoltageTimer > 3000){ //1s
                for(int i=1; i<=5; i++)
                    ReadVoltage(i);
                WaitTimer = 0;
                runtimedata.Workindex[STATUS] = STATUS_CHECK_VOLTAGE;
            }
            break;
        case STATUS_CHECK_VOLTAGE:
            if(CheckVoltageMsg()){
                runtimedata.Workindex[STATUS] = STATUS_READ_WATTAGE;
                runtimedata.WattageReadData = -1;
                runtimedata.ReadWattageRetryTimes = 0;
            }
            if(WaitTimer > HMI_CMD_WAIT_TIME)
                runtimedata.Workindex[STATUS] = STATUS_STOP;
            break;
        case STATUS_READ_WATTAGE:   //開始後15秒讀取
            if(ReadVoltageTimer > 15000){ //15s
                modbuscmd->SendWattageCommand();
                ReadVoltageTimer = 0;
                runtimedata.ReadWattageRetryTimes++;
                if(runtimedata.ReadWattageRetryTimes >= 2){
                    cmd_port->println("Read Wattage Out Retry 2 times");
                    runtimedata.WattageReadData = 0;
                    runtimedata.Workindex[STATUS] = STATUS_CHECK_WATTAGE;
                }
            }
            if(runtimedata.WattageReadData != -1){
                WaitTimer = 0;
                runtimedata.Workindex[STATUS] = STATUS_CHECK_WATTAGE;
            }
            break;
        case STATUS_CHECK_WATTAGE:
            if(CheckWattageMsg()){
                WaitTimer = 0;
                runtimedata.Workindex[STATUS] = STATUS_CHECK_END;
            }
            if(WaitTimer > HMI_CMD_WAIT_TIME)
                runtimedata.Workindex[STATUS] = STATUS_STOP;
            break;
        case STATUS_CHECK_END:   //check end
            if(CheckStartEnd(0)){
                buzzerPlay(300);
                runtimedata.Workindex[STATUS] = STATUS_STOP;
            }
            if(WaitTimer > HMI_CMD_WAIT_TIME)
                runtimedata.Workindex[STATUS] = STATUS_STOP;
            break;
    }
}

bool CheckVoltageMsg()
{
    bool result = false;
    if(CommunicationTimer > 1000){
        CommunicationTimer = 0;
        cmd_port->println("(int)runtimedata.RealV_Bus: " + String((long)runtimedata.RealV_Bus));
        cmd_port->println("(int)runtimedata.RealV_15: " + String((int)runtimedata.RealV_15));
        cmd_port->println("(int)runtimedata.RealV_3_3: " + String((int)runtimedata.RealV_3_3));
        cmd_port->println("(int)runtimedata.RealV_5: " + String((int)runtimedata.RealV_5));
        cmd_port->println("(int)runtimedata.RealV_12: " + String((int)runtimedata.RealV_12));
        hmicmd->Request_Voltage_Result(
            (long)runtimedata.RealV_Bus,
            (int)runtimedata.RealV_15,
            (int)runtimedata.RealV_3_3,
            (int)runtimedata.RealV_5,
            (int)runtimedata.RealV_12
        );
    }
    else{
        if(hmicmd->Receive_HMI_CMD == 1){
            result = true;
            hmicmd->Receive_HMI_CMD = -1;
        }
    }
    return result;
}
bool CheckWattageMsg()
{
    bool result = false;
    if(CommunicationTimer > 1000){
        CommunicationTimer = 0;
        hmicmd->Request_Wattage_Result(runtimedata.WattageReadData);
    }
    else{
        if(hmicmd->Receive_HMI_CMD == 2){
            result = true;
            hmicmd->Receive_HMI_CMD = -1;
        }
    }
    return result;
}

bool CheckStartEnd(uint8_t StartEnd)
{
    bool result = false;
    if(CommunicationTimer > 1000){
        CommunicationTimer = 0;
        hmicmd->Request_StartStop(StartEnd);     
    }
    else{
        if(hmicmd->Receive_HMI_CMD == 3){
            result = true;
            hmicmd->Receive_HMI_CMD = -1;
        }
    }
    return result;
}
bool CheckPing(void)
{
    bool result = false;
    if(CommunicationTimer > 500){
        CommunicationTimer = 0;
        hmicmd->Request_Ping();        
    }
    else{
        if(hmicmd->Receive_HMI_CMD == 0){
            result = true;
            hmicmd->Receive_HMI_CMD = -1;
        }
    }
    return result;
}

bool preStartBtnState = false;
bool CheckBtn(void)
{
    bool result = false;
    uint8_t startbtn = getInput(InStartBtn);
    if((startbtn && !preStartBtnState))
    {
        cmd_port->println("startbtn: " + String(startbtn) + ", " + String(preStartBtnState));
        buzzerPlay(300);
        hmicmd->Request_StartStop(1);
        result = true;
    }
    preStartBtnState = startbtn;
    return result;
}

void ReadVoltage(uint8_t Num)
{
    switch(Num)
    {
        case 0:
            runtimedata.VoltageReadData[Num] = analogRead(ADC_PWMPin[ADC_V_0]);
            delay(1000);
            cmd_port->println("V_0: " + String(runtimedata.VoltageReadData[Num]));
            runtimedata.V_0 = runtimedata.VoltageReadData[Num] * (5.0 / 1023.0);
            runtimedata.RealV_0 = runtimedata.V_0 * (1.0);
            cmd_port->println("V_0 Real Voltage: " + String(runtimedata.RealV_0));
            runtimedata.RealV_0 *= 100;
            break;
        case 1:
            analogWrite(ADC_PWMPin[Relay], 0);
            delay(1000);
            runtimedata.VoltageReadData[Num] = analogRead(ADC_PWMPin[ADC_V_Bus]);
            cmd_port->println("V_Bus: " + String(runtimedata.VoltageReadData[Num]));
            runtimedata.V_Bus = runtimedata.VoltageReadData[Num] * (5.0 / 1023.0);
            cmd_port->println("V_Bus in 5V: " + String(runtimedata.V_Bus));
            runtimedata.RealV_Bus = runtimedata.V_Bus * (148.0);
            cmd_port->println("V_Bus Real Voltage: " + String(runtimedata.RealV_Bus));
            runtimedata.RealV_Bus *= 100;
            break;
        case 2:
            analogWrite(ADC_PWMPin[Relay], 0);
            delay(1000);
            runtimedata.VoltageReadData[Num] = analogRead(ADC_PWMPin[ADC_V_15]);
            cmd_port->println("V_15: " + String(runtimedata.VoltageReadData[Num]));
            runtimedata.V_15 = runtimedata.VoltageReadData[Num] * (5.0 / 1023.0);
            cmd_port->println("V_15 in 5V: " + String(runtimedata.V_15));
            runtimedata.RealV_15 = runtimedata.V_15 * (6.0);
            cmd_port->println("V_15 Real Voltage: " + String(runtimedata.RealV_15));
            runtimedata.RealV_15 *= 10;
            break;
        case 3:
            analogWrite(ADC_PWMPin[Relay], 0);
            delay(1000);
            runtimedata.VoltageReadData[Num] = analogRead(ADC_PWMPin[ADC_V_3_3]);
            cmd_port->println("V_3_3: " + String(runtimedata.VoltageReadData[Num]));
            runtimedata.V_3_3 = runtimedata.VoltageReadData[Num] * (5.0 / 1023.0);
            cmd_port->println("V_3_3 in 5V: " + String(runtimedata.V_3_3));
            runtimedata.RealV_3_3 = runtimedata.V_3_3 * (1.32);
            cmd_port->println("V_3_3 Real Voltage: " + String(runtimedata.RealV_3_3));
            runtimedata.RealV_3_3 *= 10;
            break;
        case 4:
            analogWrite(ADC_PWMPin[Relay], 1023);
            delay(1000);
            runtimedata.VoltageReadData[Num] = analogRead(ADC_PWMPin[ADC_V_5]);
            cmd_port->println("V_5: " + String(runtimedata.VoltageReadData[Num]));
            runtimedata.V_5 = runtimedata.VoltageReadData[Num] * (5.0 / 1023.0);
            cmd_port->println("V_5 in 5V: " + String(runtimedata.V_5));
            runtimedata.RealV_5 = runtimedata.V_5 * (2.0);
            cmd_port->println("V_5 Real Voltage: " + String(runtimedata.RealV_5));
            runtimedata.RealV_5 *= 10;
            break;
        case 5:
            analogWrite(ADC_PWMPin[Relay], 1023);
            delay(1000);
            runtimedata.VoltageReadData[Num] = analogRead(ADC_PWMPin[ADC_V_12]);
            cmd_port->println("V_12: " + String(runtimedata.VoltageReadData[Num]));
            runtimedata.V_12 = runtimedata.VoltageReadData[Num] * (5.0 / 1023.0);
            cmd_port->println("V_12 in 5V: " + String(runtimedata.V_12));
            runtimedata.RealV_12 = runtimedata.V_12 * (4.8);
            cmd_port->println("V_12 Real Voltage: " + String(runtimedata.RealV_12));
            runtimedata.RealV_12 *= 10;
            break;
    }
}

void buzzerPlay(int playMS)
{
  digitalWrite(BUZZ, HIGH);
  delay(playMS);
  digitalWrite(BUZZ, LOW);
}


