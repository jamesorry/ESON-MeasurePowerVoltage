#include <avr/wdt.h>
#include "SoftwareSerial.h"
#include "UserCommand.h"
#include "EEPROM_Function.h"
#include "MainProcess.h"
#include "hmi.h"
#include "ModbusRTU_protocol.h"
#include "HMI_Command.h"

#define USER_COMMAND_DEBUG  1

extern HardwareSerial *cmd_port;
extern MainDataStruct maindata;
extern RuntimeStatus runtimedata;
extern DigitalIO digitalio;
extern ModbusRTU_Protocol *modbuscmd;


CMD g_cmdFunc[] = {
//在這新增function name 以及所呼叫的function
	{"adc", getAdc},
	{"getgpio", getGpio},
	{"setgpio", setGpio},
	{"reset", resetArduino},
	{"getmicros", getMicros},
	{"ver", cmd_CodeVer},
	{"echoon", echoOn},
	{"echooff", echoOff},
	{"reset", resetArduino},
	{"getmicros", getMicros},
	{"out", cmdOutput},
	{"in", cmdInput},
	{"TestModbusRTU",cmdTestModbusRTU},
	{"SetRPM",Cmd_SetRPM},	
	{"SetStep",cmd_SetStep},
	{"Absolute_position",cmd_Absolute_position},
	{"Emergency_Stop",cmd_Emergency_Stop},
	{"Search_Home",cmd_Search_Home},
	{"SendCommandTest",cmd_SendCommandTest},
	{"?", showHelp}
};


String g_inputBuffer0 = "";
String* g_inputBuffer = NULL;
String g_cmd = "";
String g_arg = "";

bool g_echoOn = true;

uint32_t targetcnt = 0;

bool getNextArg(String &arg)
{
	if (g_arg.length() == 0)
		return false;
	if (g_arg.indexOf(" ") == -1)
	{
		arg = g_arg;
		g_arg.remove(0);
	}
	else
	{
		arg = g_arg.substring(0, g_arg.indexOf(" "));
		g_arg = g_arg.substring(g_arg.indexOf(" ") + 1);
	}
	return true;
}

void resetArduino(void)
{
	wdt_enable(WDTO_500MS);
	while (1);
}
void getMicros(void)
{
	cmd_port->println(String("micros:") + micros());
}

void showHelp(void)
{
	int i;

	cmd_port->println("");
	for (i = 0; i < (sizeof(g_cmdFunc) / sizeof(CMD)); i++)
	{
		cmd_port->println(g_cmdFunc[i].cmd);
	}
}

void getAdc(void)
{
	String arg1;
	int analogPin;
	int value;

	if (!getNextArg(arg1))
	{
		cmd_port->println("No parameter");
		return;
	}
	analogPin = arg1.toInt();
	value = analogRead(analogPin);
	cmd_port->print("ADC_");
	cmd_port->print(analogPin);
	cmd_port->print(" : ");
	cmd_port->println(value);
}

void getGpio(void)
{
  String arg1, arg2;
  int digitalPin, pullUp;
  int value;

  if (!getNextArg(arg1))
  {
    cmd_port->println("No parameter");
    return;
  }
  if (!getNextArg(arg2))
  {
    pullUp = 0;
  }
  else
  {
    pullUp = arg2.toInt();
  }
  digitalPin = arg1.toInt();
  if (arg2.compareTo("na") == 0)
  {
    cmd_port->println("pin mode keep original");
  }
  else
  {
    if (pullUp)
    {
      cmd_port->println("pull-up");
      pinMode(digitalPin, INPUT_PULLUP);
    }
    else
    {
      cmd_port->println("no-pull");
      pinMode(digitalPin, INPUT);
    }
  }

  cmd_port->print("GPIO:");
  cmd_port->println(arg1);

  value = digitalRead(digitalPin);

  cmd_port->print("input value:");
  cmd_port->println(value);
}

void setGpio(void)
{
  String arg1, arg2;
  int digitalPin;
  int value;

  if (!getNextArg(arg1))
  {
    cmd_port->println("No parameter 1");
    return;
  }
  if (!getNextArg(arg2))
  {
    cmd_port->println("No parameter 2");
    return;
  }
  digitalPin = arg1.toInt();
  value = arg2.toInt();

  cmd_port->print("GPIO:");
  cmd_port->println(arg1);
  cmd_port->print("level:");
  cmd_port->println(arg2);

  digitalWrite(digitalPin, value ? HIGH : LOW);
  pinMode(digitalPin, OUTPUT);
}

void echoOn(void)
{
  g_echoOn = true;
}

void echoOff(void)
{
  g_echoOn = false;
}

void cmd_CodeVer(void)
{
  cmd_port->println(VERSTR);
}

void cmdOutput(void)
{
	String arg1, arg2;
	int digitalPin;
	int value;

	if (!getNextArg(arg1))
	{
		cmd_port->println("No parameter 1");
		return;
	}
	if (!getNextArg(arg2))
	{
		cmd_port->println("No parameter 2");
		return;
	}
	digitalPin = arg1.toInt();
	value = arg2.toInt();

	cmd_port->print("PIN index:");
	cmd_port->println(arg1);
	cmd_port->print("level:");
	cmd_port->println(arg2);

	setOutput((uint8_t)digitalPin, (uint8_t)(value ? HIGH : LOW));
	cmd_port->println("");
}

void cmdInput(void)
{
	String arg1;
	unsigned long pinindex;

	getNextArg(arg1);
	if( (arg1.length()==0))
	{
		cmd_port->println("Please input enough parameters");
		return;
	}
	pinindex = arg1.toInt();
	cmd_port->println("Sensor: " + String(getInput(pinindex)));
}

void cmdTestModbusRTU(void)
{
	String arg1;
	
	uint16_t recData; //16bit
	uint8_t	data[32]; //8bit
//	calc_crc(buffer_clrAlarm, sizeof(buffer_clrAlarm)-2);
//	Serial.println(calc_crc(buffer_clrAlarm, sizeof(buffer_clrAlarm)-2),HEX);
//	recData = calc_crc(buffer_clrAlarm, sizeof(buffer_clrAlarm)-2);
	data[0] = recData & 0xff; //取後8bit的值
	data[1] = recData >> 8;   //向右shift 8bit
	Serial.print("HIGH: ");
	Serial.println(data[0],HEX);
	Serial.print("LOW: ");
	Serial.println(data[1],HEX);
	data[2] = lowByte(recData);
	data[3] = highByte(recData);	
	Serial.print("HIGH: ");
	Serial.println(data[2],HEX);
	Serial.print("LOW: ");
	Serial.println(data[3],HEX);
	Serial.println();

//	sendData(buffer_clrAlarm, sizeof(buffer_clrAlarm) );

}

void Cmd_SetRPM(void)
{
	String arg1;
	long value;

	if (!getNextArg(arg1))
	{
	  cmd_port->println("No parameter 1");
	  return;
	}
	value = arg1.toInt();
	modbuscmd->Set_RPM(value);
}

void cmd_SetStep(void)
{
	String arg1;
	long value;

	if (!getNextArg(arg1))
	{
	  cmd_port->println("No parameter 1");
	  return;
	}
	value = arg1.toInt();
	modbuscmd->Set_Step(value);
}

void cmd_Absolute_position(void)
{
	modbuscmd->Absolute_position();
}

void cmd_Emergency_Stop(void)
{
	modbuscmd->Emergency_Stop();
}

void cmd_Search_Home(void)
{
	modbuscmd->Search_Home();
}

void cmd_SendCommandTest(void)
{
	String arg1,arg2,arg3,arg4,arg5;
	long ID, FunctionCode, Addr, Datacnt, value;
	if (!getNextArg(arg1))
	{
	  cmd_port->println("No parameter ID");
	  return;
	}
	if (!getNextArg(arg2))
	{
	  cmd_port->println("No parameter FunctionCode");
	  return;
	}
	if (!getNextArg(arg3))
	{
	  cmd_port->println("No parameter Addr");
	  return;
	}
	if (!getNextArg(arg4))
	{
	  cmd_port->println("No parameter Datacnt");
	  return;
	}
	if (!getNextArg(arg5))
	{
	  cmd_port->println("No parameter value");
	  return;
	}
	ID = arg1.toInt();
	FunctionCode = arg2.toInt();
	Addr = arg3.toInt();
	Datacnt = arg4.toInt();
	value = arg5.toInt();
	modbuscmd->Send_Command(ID, FunctionCode, (uint32_t)Addr, (uint32_t)Datacnt, (uint32_t)value);
}



uint8_t UserCommWorkindex = 0;

uint32_t UserCommandTimeCnt = 0;

void UserCommand_Task(void)
{
  int i, incomingBytes, ret, cmdPortIndex;
  char data[2] = {0};

  switch(UserCommWorkindex)
  {
    case 0:
    {
      
      if(cmd_port->available())
      {
        g_inputBuffer = &g_inputBuffer0;
        UserCommWorkindex ++;
        UserCommandTimeCnt = millis();
      }
      break;
    }
    case 1:
    {
      if((millis() - UserCommandTimeCnt) > 50)
        UserCommWorkindex ++;
      break;
    }
    case 2:
    {
      if ( incomingBytes = cmd_port->available() )
      {

      cmd_port->println("cmd_port datalen: " + String(incomingBytes));

      for ( i = 0; i < incomingBytes; i++ )
      {
        ret = cmd_port->read();
        if ( (ret >= 0x20) && (ret <= 0x7E) || (ret == 0x0D) || (ret == 0x0A) )
        {
        data[0] = (char)ret;
        (*g_inputBuffer) += data;
        if (g_echoOn)
        {
          if ( (data[0] != 0x0D) && (data[0] != 0x0A) )
          cmd_port->write(data);
        }
        }
        else if (ret == 0x08)
        {
        if (g_inputBuffer->length())
        {
          g_inputBuffer->remove(g_inputBuffer->length() - 1);
          if (g_echoOn)
          {
          data[0] = 0x08;
          cmd_port->write(data);
          }
        }
        }
      }
      if (g_inputBuffer->indexOf('\r') == -1)
      {
        if (g_inputBuffer->indexOf('\n') == -1)
        return;
      }
      g_inputBuffer->trim();
      while (g_inputBuffer->indexOf('\r') != -1)
        g_inputBuffer->remove(g_inputBuffer->indexOf('\r'), 1);
      while (g_inputBuffer->indexOf('\n') != -1)
        g_inputBuffer->remove(g_inputBuffer->indexOf('\n'), 1);
      while (g_inputBuffer->indexOf("  ") != -1)
        g_inputBuffer->remove(g_inputBuffer->indexOf("  "), 1);
    
      cmd_port->println();
    
      if (g_inputBuffer->length())
      {
        g_arg.remove(0);
        if (g_inputBuffer->indexOf(" ") == -1)
        g_cmd = (*g_inputBuffer);
        else
        {
        g_cmd = g_inputBuffer->substring(0, g_inputBuffer->indexOf(" "));
        g_arg = g_inputBuffer->substring(g_inputBuffer->indexOf(" ") + 1);
        }
        for (i = 0; i < (sizeof(g_cmdFunc) / sizeof(CMD)); i++)
        {
        //if(g_cmd==g_cmdFunc[i].cmd)
        if (g_cmd.equalsIgnoreCase(g_cmdFunc[i].cmd))
        {
          g_cmdFunc[i].func();
          cmd_port->print("ARDUINO>");
          break;
        }
        else if (i == (sizeof(g_cmdFunc) / sizeof(CMD) - 1))
        {
          cmd_port->println("bad command !!");
          cmd_port->print("ARDUINO>");
        }
        }
        *g_inputBuffer = "";
      }
      else
      {
        cmd_port->print("ARDUINO>");
      }
      UserCommWorkindex = 0;
      break;
    }
  }

  }
}

