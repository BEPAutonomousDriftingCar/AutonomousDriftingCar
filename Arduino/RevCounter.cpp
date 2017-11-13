/*
RevCounter.cpp
Maarten Kleijwegt
st.maartenk@gmail.com

Copyright (c) 2017 Maarten Kleijwegt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "RevCounter.h"
#include "Arduino.h"

RevCounter::RevCounter(){}
void RevCounter::begin() {
	
//configure Flextimer 1
FTM1_MODE = 0x05; //set write-protect disable (WPDIS) bit to modify other registers
//FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1(no restriction FTM)
FTM1_SC = 0x00; //set status/control to zero = disable all internal clock sources
FTM1_CNTIN = 0;
FTM1_CNT = 0x0000; //reset count to zero
FTM1_MOD = 0xFFFF; //max modulus = 9999 (gives count = 10000 on roll-over) 

//configure Flextimer 2
FTM2_MODE = 0x05; //set write-protect disable (WPDIS) bit to modify other registers
//FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1(no restriction FTM)
FTM2_SC = 0x00; //set status/control to zero = disable all internal clock sources
FTM2_CNTIN = 0;
FTM2_CNT = 0x0000; //reset count to zero
FTM2_MOD = 0xFFFF; //max modulus = 9999 (gives count = 10000 on roll-over) 


//configure Teensy port pins
//PORTD_PCR6 |= 0x300;  //MUx = Set uart to different channel
//PORTD_PCR7 |= 0x300;
PORTB_PCR16 |= 0x400;  //MUX = FTM_CLKIN0 ALT4 on Chip Pin 95 = Teensy Pin 0
PORTB_PCR17 |= 0x400;  //MUX = FTM_CLKIN1 ALT4 on Chip Pin 95 = Teensy Pin 1

//initialise SIM_SOPT4
SIM_SOPT4 = 0x04000000; //uses FTM_CLKIN0 and FTM_CLKIN1 for external clock signals

//set counter to use external clocks
FTM1_SC = 0x18; // (Note – FTM1_SC [TOF=0 TOIE=0 CPWMS=0 CLKS=11 (external clocks selected) PS=000 [no prescale divide])
FTM2_SC = 0x18; // (Note – FTM2_SC [TOF=0 TOIE=0 CPWMS=0 CLKS=11 (external clocks selected) PS=000 [no prescale divide])
}


