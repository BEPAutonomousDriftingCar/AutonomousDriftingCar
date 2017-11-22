#include "RevCounter.h"
int FTM1CountOVFlow;
int FTM1Ch0CountValue;
RevCounter RevCounter;
//int rpm;



//(1) Setup statements
void setup() {
  //Set up Serial2 connection to computer
 Serial2.begin(9600);
 delay(500);
 Serial2.println("All set up, lets begin");


 
	RevCounter.begin();
 delay(500);

pinMode(13, OUTPUT);
digitalWrite(13, HIGH);





}

//(3) main loop statements
void loop() {
//read FTM1 Ch0 value if valid
FTM1Ch0CountValue = FTM1_CNT; //read CH0 value
//rpm =analogRead(9)*3300/1024;
Serial2.print(RevCounter.readCounter(0));
Serial2.print(" ");
Serial2.print(RevCounter.readCounter(1));
Serial2.print(" ");
Serial2.print(RevCounter.readCounter(2));
Serial2.print(" ");
Serial2.println(RevCounter.readCounter(3));
//Serial2.print(" ");
//Serial2.println(rpm);
delay(499);
}
