// Simple input path for RC Car

#include <Servo.h>//Using servo library to control ESC
Servo esc; //Creating a servo class with name as esc
Servo steering; //Creating a servo class with name as steering

int esc1 = 1500; //Start value for esc
int esc2 = 1580; //Forward
int esc3 = 1650; //Fast forward
int escbrake = 1490; //Braking

int steering1 = 90; //Start value for steering
int steering2 = 55;

int var; //variable for amount of loops

void setup() {
  // put your setup code here, to run once:

esc.attach(8); //Specify the esc signal pin, here as D8
esc.writeMicroseconds(1000); //initialize the esc signal to 1000
steering.attach(9); //Specify the steering signal pin, here D9.
  Serial.begin(9600);
}

void loop() { 
while(var<1){  
  var++;
  
esc.writeMicroseconds(esc1); //input esc 1500
  
  delay(100);  
  
steering.write(steering1); //input steering 90

 delay(500);

esc.writeMicroseconds(esc2); //input esc 1580
 
 delay(1000);

esc.writeMicroseconds(esc3); //input esc 1640
steering.write(steering2); //input steering 55
 
 delay(1500); //drift time

esc.writeMicroseconds(esc2); //input esc 1580

 delay(1000);
 
esc.writeMicroseconds(esc1);
delay(100);
esc.writeMicroseconds(escbrake);

delay(1000);

esc.writeMicroseconds(esc1);
steering.write(steering1);

}
}
