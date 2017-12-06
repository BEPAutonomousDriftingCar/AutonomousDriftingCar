/*
 *  Interrupt and PWM utilities for 16 bit Timer3 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified Oct 2009 by Dan Clemens to work with timer3 of the ATMega1280 or Arduino Mega
 *  Modified April 2012 by Paul Stoffregen
 *  Modified again, June 2014 by Paul Stoffregen
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#include "TimerFour.h"

TimerFour Timer4;              // preinstatiate

unsigned short TimerFour::pwmPeriod = 0;
unsigned char TimerFour::clockSelectBits = 0;
void (*TimerFour::isrCallback)() = TimerFour::isrDefaultUnused;


#if defined(__arm__) && defined(CORE_TEENSY)
void ftm3_isr(void)
{
  uint32_t sc = FTM3_SC;
  #ifdef KINETISL
  if (sc & 0x80) FTM3_SC = sc;
  #else
  if (sc & 0x80) FTM3_SC = sc & 0x7F;
  #endif
  Timer4.isrCallback();
}

#endif

void TimerFour::isrDefaultUnused()
{
}
