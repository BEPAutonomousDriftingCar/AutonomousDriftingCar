#include "RevCounter.h"
int FTM1CountOVFlow;
int FTM1Ch0CountValue;
RevCounter RevCounter;
//int rpm;



//(1) Setup statements
void setup() {
  //Set up serial connection to computer
 Serial.begin(9600);
 delay(500);
 Serial.println("All set up, lets begin");


 
	RevCounter.begin();
 Serial.println(TPM1_SC);

pinMode(13, OUTPUT);
digitalWrite(13, HIGH);





}

//(3) main loop statements
void loop() {
//read FTM1 Ch0 value if valid
FTM1Ch0CountValue = FTM1_CNT; //read CH0 value
//rpm =analogRead(9)*3300/1024;
Serial.print(FTM1_CNT);
Serial.print(" ");
Serial.print(FTM2_CNT);
Serial.print(" ");
Serial.print(TPM1_CNT);
Serial.print(" ");
Serial.println(TPM2_CNT);
//Serial.print(" ");
//Serial.println(rpm);
delay(499);
}
