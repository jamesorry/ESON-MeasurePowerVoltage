#include "Arduino.h"  

extern HardwareSerial *cmd_port;

unsigned long g_milliTimeoutBegin = 0;
bool g_timeoutEnable = true;


void setTimeoutBegin(void)
{
	g_milliTimeoutBegin = millis();
}
bool isTimeoutMilliSec(unsigned long milliSecTimeout)
{
	unsigned long currentMilliSec;

	if(!g_timeoutEnable)
		return false;
	currentMilliSec = millis();
	if( (currentMilliSec-g_milliTimeoutBegin)>=milliSecTimeout )
	{
		return true;
	}
	else
	{
		return false;
	}
}

void TimerInit(unsigned char timer, unsigned long microSec, unsigned short ocrna, unsigned int tccrnb)
{
	//noInterrupts()
	switch(timer)
	{
	case 1:
		TIMSK1 &= ~(1<<OCIE1A);	//disable timer compare interrupt
	
		TCCR1A = 0;
		TCNT1 = 0;
	
		OCR1A = ocrna;
		TCCR1B = tccrnb;
	
		TIMSK1 |= (1<<OCIE1A);	//enable timer compare interrupt
		break;
/*		
	case 3:
		TIMSK3 &= ~(1<<OCIE3A);	//disable timer compare interrupt
	
		TCCR3A = 0;
		TCNT3 = 0;
	
		OCR3A = ocrna;
		TCCR3B = tccrnb;
	
		TIMSK3 |= (1<<OCIE3A);	//enable timer compare interrupt
		break;
*/		
	default:
		break;
	}
	//interrupts();
}
void TimerInit(unsigned char timer, unsigned long microSec)
{
/*
	(2 x prescaler) / 16MHz <= T <= (65535 x prescaler) / 16MHz

	1 prescaler
	0.125 micro sec <= T <= 4.0959375 milli sec

	8 prescaler
	1 micro sec <= T <= 32.7675 milli sec

	64 prescaler
	8 micro sec <= T <= 262.140 milli sec

	256 prescaler
	32 micro sec <= T <= 1.048560 sec

	1024 prescaler
	128 micro sec <= T <= 4.19424 sec
*/

	//noInterrupts()
	switch(timer)
	{
	case 1:
		TIMSK1 &= ~(1<<OCIE1A);	//disable timer compare interrupt
		TCCR1A = 0;
		TCNT1 = 0;

		//OCR1A output compare register 16MHz*sec/256 = 16*microSec/256
		if( microSec<=4095 )
		{//use 1 prescaler
			OCR1A = (unsigned short)(microSec*16);
			TCCR1B = (1<<WGM12) | (1<<CS10);	//CTC mode , 1 prescaler
		}
		else if( microSec<=32767 )
		{//use 8 prescaler
			OCR1A = (unsigned short)(microSec*16/8);
			TCCR1B = (1<<WGM12) | (1<<CS11);	//CTC mode , 8 prescaler
		}
		else if( microSec<=262140 )
		{//use 64 prescaler
			OCR1A = (unsigned short)(microSec*16/64);
			TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);	//CTC mode , 64 prescaler
		}
		else if( microSec<=1048560 )
		{//use 256 prescaler
			OCR1A = (unsigned short)(microSec*16/256);
			TCCR1B = (1<<WGM12) | (1<<CS12);	//CTC mode , 256 prescaler
		}
		else if( microSec<=4194240 )
		{//use 1024 prescaler
			OCR1A = (unsigned short)(microSec*16/1024);
			TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10);	//CTC mode , 1024 prescaler
		}
		else
		{
		}

		TIMSK1 |= (1<<OCIE1A);	//enable timer compare interrupt
		break;
		
	case 3:
		TIMSK3 &= ~(1<<OCIE3A);	//disable timer compare interrupt
		TCCR3A = 0;
		TCNT3 = 0;

		//OCR3A output compare register 16MHz*sec/256 = 16*microSec/256
		if( microSec<=4095 )
		{//use 1 prescaler
			OCR3A = (unsigned short)(microSec*16);
			TCCR3B = (1<<WGM32) | (1<<CS30);	//CTC mode , 1 prescaler
		}
		else if( microSec<=32767 )
		{//use 8 prescaler
			OCR3A = (unsigned short)(microSec*16/8);
			TCCR3B = (1<<WGM32) | (1<<CS31);	//CTC mode , 8 prescaler
		}
		else if( microSec<=262140 )
		{//use 64 prescaler
			OCR3A = (unsigned short)(microSec*16/64);
			TCCR3B = (1<<WGM32) | (1<<CS31) | (1<<CS30);	//CTC mode , 64 prescaler
		}
		else if( microSec<=1048560 )
		{//use 256 prescaler
			OCR3A = (unsigned short)(microSec*16/256);
			TCCR3B = (1<<WGM32) | (1<<CS32);	//CTC mode , 256 prescaler
		}
		else if( microSec<=4194240 )
		{//use 1024 prescaler
			OCR3A = (unsigned short)(microSec*16/1024);
			TCCR3B = (1<<WGM32) | (1<<CS32) | (1<<CS30);	//CTC mode , 1024 prescaler
		}
		else
		{
		}

		TIMSK3 |= (1<<OCIE3A);	//enable timer compare interrupt
		break;
		
	default:
		break;
	}

	//interrupts();
}
void TimerDeinit(unsigned char timer)
{
	switch(timer)
	{
	case 1:
		TIMSK1 &= ~(1<<OCIE1A);	//disable timer compare interrupt
		break;
/*		
	case 3:
		TIMSK3 &= ~(1<<OCIE3A);	//disable timer compare interrupt
		break;
*/		
	default:
		break;
	}
}
