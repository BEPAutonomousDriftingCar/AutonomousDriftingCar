/*
 *  Interrupt and PWM utilities for 16 bit Timer3 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified April 2012 by Paul Stoffregen - portable to other AVR chips, use inline functions
 *  Modified again, June 2014 by Paul Stoffregen - support Teensy 3.1 & even more AVR chips
 *
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#ifndef TimerFour_h_
#define TimerFour_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "config/known_16bit_timers.h"

#define TIMER4_RESOLUTION 65536UL  // Timer3 is 16 bit


// Placing nearly all the code in this .h file allows the functions to be
// inlined by the compiler.  In the very common case with constant values
// the compiler will perform all calculations and simply write constants
// to the hardware registers (for example, setPeriod).


class TimerFour
{






#if defined(__arm__) && defined(CORE_TEENSY)

#if defined(KINETISK)
#define F_TIMER F_BUS
#elif defined(KINETISL)
#define F_TIMER (F_PLL/2)
#endif

  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {
	const unsigned long cycles = (F_TIMER / 2000000) * microseconds;

  /*
  // This code does not work properly in all cases :(
  // https://github.com/PaulStoffregen/TimerOne/issues/17 
  if (cycles < TIMER3_RESOLUTION * 16) {
    if (cycles < TIMER3_RESOLUTION * 4) {
      if (cycles < TIMER3_RESOLUTION) {
        clockSelectBits = 0;
        pwmPeriod = cycles;
      }else{
        clockSelectBits = 1;
        pwmPeriod = cycles >> 1;
      }
    }else{
      if (cycles < TIMER3_RESOLUTION * 8) {
        clockSelectBits = 3;
        pwmPeriod = cycles >> 3;
      }else{
        clockSelectBits = 4;
        pwmPeriod = cycles >> 4;
      }
    }
  }else{
    if (cycles > TIMER3_RESOLUTION * 64) {
      if (cycles > TIMER3_RESOLUTION * 128) {
        clockSelectBits = 7;
        pwmPeriod = TIMER3_RESOLUTION - 1;
      }else{
        clockSelectBits = 7;
        pwmPeriod = cycles >> 7;
      }
    }else{
      if (cycles > TIMER3_RESOLUTION * 32) {
        clockSelectBits = 6;
        pwmPeriod = cycles >> 6;
      }else{
        clockSelectBits = 5;
        pwmPeriod = cycles >> 5;
      }
    }
  }
  */
	if (cycles < TIMER4_RESOLUTION) {
		clockSelectBits = 0;
		pwmPeriod = cycles;
	} else
	if (cycles < TIMER4_RESOLUTION * 2) {
		clockSelectBits = 1;
		pwmPeriod = cycles >> 1;
	} else
	if (cycles < TIMER4_RESOLUTION * 4) {
		clockSelectBits = 2;
		pwmPeriod = cycles >> 2;
	} else
	if (cycles < TIMER4_RESOLUTION * 8) {
		clockSelectBits = 3;
		pwmPeriod = cycles >> 3;
	} else
	if (cycles < TIMER4_RESOLUTION * 16) {
		clockSelectBits = 4;
		pwmPeriod = cycles >> 4;
	} else
	if (cycles < TIMER4_RESOLUTION * 32) {
		clockSelectBits = 5;
		pwmPeriod = cycles >> 5;
	} else
	if (cycles < TIMER4_RESOLUTION * 64) {
		clockSelectBits = 6;
		pwmPeriod = cycles >> 6;
	} else
	if (cycles < TIMER4_RESOLUTION * 128) {
		clockSelectBits = 7;
		pwmPeriod = cycles >> 7;
	} else {
		clockSelectBits = 7;
		pwmPeriod = TIMER4_RESOLUTION - 1;
	}

	uint32_t sc = FTM3_SC;
	FTM3_SC = 0;
	FTM3_MOD = pwmPeriod;
	FTM3_SC = FTM_SC_CLKS(1) | FTM_SC_CPWMS | clockSelectBits | (sc & FTM_SC_TOIE);
    }

    //****************************
    //  Run Control
    //****************************
    void start() __attribute__((always_inline)) {
	stop();
	FTM3_CNT = 0;
	resume();
    }
    void stop() __attribute__((always_inline)) {
	FTM3_SC = FTM3_SC & (FTM_SC_TOIE | FTM_SC_CPWMS | FTM_SC_PS(7));
    }
    void restart() __attribute__((always_inline)) {
	start();
    }
    void resume() __attribute__((always_inline)) {
	FTM3_SC = (FTM3_SC & (FTM_SC_TOIE | FTM_SC_PS(7))) | FTM_SC_CPWMS | FTM_SC_CLKS(1);
    }

    
    //****************************
    //  Interrupt Function
    //****************************
    void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	FTM3_SC |= FTM_SC_TOIE;
	NVIC_ENABLE_IRQ(IRQ_FTM3);
    }
    void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(isr);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	FTM3_SC &= ~FTM_SC_TOIE;
	NVIC_DISABLE_IRQ(IRQ_FTM3);
    }
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;

#undef F_TIMER

#endif
};

extern TimerFour Timer4;

#endif

