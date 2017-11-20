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

//Set up System options
SIM_SOPT4 |= 0x04000000; //uses FTM_CLKIN0 and FTM_CLKIN1 for external clock signals
SIM_SCGC2 |= 0x600; // Set Gate clock select to enable TPM2 and TPM1

	
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

//configure TPM1
TPM1_SC &= ~0x18; //set status/control to zero = disable all internal clock sources
TPM1_CNT = 0x0000; //reset count to zero
TPM1_MOD = 0xFFFF; //max modulus = 9999 (gives count = 10000 on roll-over)
TPM1_C0SC=0x00;
TPM1_C1SC=0x00; 
TPM1_QDCTRL = 0x0F; //see section 36.3.21 of ref manual for details 

//configure TPM2
TPM2_SC &= ~0x18; //set status/control to zero = disable all internal clock sources
TPM2_CNT = 0x0000; //reset count to zero
TPM2_MOD = 0xFFFF; //max modulus = 9999 (gives count = 10000 on roll-over)
TPM2_C0SC=0x00;
TPM2_C1SC=0x00; 
TPM2_QDCTRL = 0x0F; //see section 36.3.21 of ref manual for details 



//configure Teensy port pins
PORTB_PCR16 |= 0x400;  //MUX = FTM_CLKIN0 ALT4 on Chip Pin 95 = Teensy Pin 0
PORTB_PCR17 |= 0x400;  //MUX = FTM_CLKIN1 ALT4 on Chip Pin 95 = Teensy Pin 1
PORTA_PCR12 |= 0x700; //MUX = alternative function 7 on Chip Pin 28 (TPM1_QD_PHA) = Teensy Pin 3 
PORTA_PCR13 |= 0x700; //MUX = alternative function 7 on Chip Pin 29 (TPM1_QD_PHB) = Teensy Pin 4 
PORTB_PCR18 |= 0x600; //MUX = alternative function 6 on Chip Pin 97 (TPM2_QD_PHA) = Teensy Pin 29 
PORTB_PCR19 |= 0x600; //MUX = alternative function 6 on Chip Pin 98 (TPM2_QD_PHB) = Teensy Pin 30

//initialise SIM_SOPT4


//set counter to use external clocks and enable counters
FTM1_SC = 0x18; // (Note – FTM1_SC [TOF=0 TOIE=0 CPWMS=0 CLKS=11 (external clocks selected) PS=000 [no prescale divide])
FTM2_SC = 0x18; // (Note – FTM2_SC [TOF=0 TOIE=0 CPWMS=0 CLKS=11 (external clocks selected) PS=000 [no prescale divide])
TPM1_SC = 0x08;
TPM2_SC = 0x08;
}


