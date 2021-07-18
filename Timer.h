#ifndef _TIMER_H_
#define _TIMER_H_

#define TIMER_INTERVAL_MS	10
void TimerInit(unsigned char timer, unsigned long microSec, unsigned short ocrna, unsigned int tccrnb);
void TimerInit(unsigned char timer, unsigned long microSec);

#endif
