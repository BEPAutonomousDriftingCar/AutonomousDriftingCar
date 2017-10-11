#include <Servo.h> //Include Servo library in script

Servo steering; //Declare Servo name

void setup() {
  
Serial.begin(9600);
steering.attach(8); //Attach Servo to Digital Pin 8
steering.write(90); //Calibrate steering angle

}

void loop() {
  // put your main code here, to run repeatedly:
delay(100);
steering.write(90); //Fill in value between 55-135

}
