int FTM1CountOVFlow;
int FTM1Ch0CountValue;
//int rpm;

//(1) Setup statements
void setup() {
//configure Flextimer 1
FTM1_MODE = 0x05; //set write-protect disable (WPDIS) bit to modify other registers
//FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1(no restriction FTM)
FTM1_SC = 0x00; //set status/control to zero = disable all internal clock sources
FTM1_CNTIN = 0; // set initial value to zero
FTM1_CNT = 0x0000; //reset count to zero
FTM1_MOD = 9999; //max modulus = 9999 (gives count = 10000 on roll-over) 

//enable FTM1 interrupt within NVIC table
NVIC_ENABLE_IRQ(IRQ_FTM1);

//configure Teensy port pins
//PORTB_PCR18 |= 0x400;  //MUX = FTM_CLKIN0 ALT4 on Chip Pin 95 = Teensy Pin 29
PORTA_PCR12 |= 0x700; //MUX = alternative function 7 on Chip Pin 28 (FTM1_QD_PHA) = Teensy Pin 3
PORTA_PCR13 |= 0x700; //MUX = alternative function 7 on Chip Pin 29 (FTM1_QD_PHB) = Teensy Pin 4

//set flextimer quad decode mode and enable overflow interrupt
FTM1_QDCTRL = 0x0F; //see section 36.3.21 of ref manual for details
FTM1_SC = 0x40; // (Note – FTM1_SC [TOF=0 TOIE=1 CPWMS=0 CLKS=00 (internal clocks disabled) PS=000 [no prescale divide])

//Set up serial connection to computer
 Serial.begin(9600);
 Serial.println("All set up, lets begin");
 pinMode(13, OUTPUT);
 digitalWrite(13, HIGH);
}

//(2) ISR routine for FlexTimer1 Module
extern "C" void ftm1_isr(void) {
if ((FTM1_SC & FTM_SC_TOF) != 0) { //read the timer overflow flag (TOF in FTM1_SC)
FTM1_SC &= ~FTM_SC_TOF; //if set, clear overflow flag
}
FTM1CountOVFlow++; //increment overflow counter
}

//(3) main loop statements
void loop() {
//read FTM1 Ch0 value if valid
FTM1Ch0CountValue = FTM1_CNT; //read CH0 value
//rpm =analogRead(9)*3300/1024;
Serial.println(FTM1Ch0CountValue);
//Serial.print(" ");
//Serial.println(rpm);
delay(499);
}
