#include <MedianFilter.h>

MedianFilter median(8, 3000); // Setup Median Filter

#define trigPin1 2 // Define pins
#define echoPin1 3 // Define pins
#define trigPin2 4 // Define pins
#define echoPin2 5 // Define pins
#define trigPin3 6 // Define pins
#define echoPin3 7 // Define pins
#define trigPin4 8 // Define pins
#define echoPin4 9 // Define pins
/*#define trigPin5 11  // Define pins
#define echoPin5 12 // Define pins
#define trigPin6 13 // Define pins
#define echoPin6 14 // Define pins */

long duration, distance, Sensor1,Sensor2,Sensor3,Sensor4, Sensor5, Sensor6;

void setup() {
Serial.begin (9600);
pinMode(trigPin1, OUTPUT); // Define input/output pins
pinMode(echoPin1, INPUT); // Define input/output pins
pinMode(trigPin2, OUTPUT); // Define input/output pins
pinMode(echoPin2, INPUT); // Define input/output pins
pinMode(trigPin3, OUTPUT); // Define input/output pins
pinMode(echoPin3, INPUT); // Define input/output pins
pinMode(trigPin4, OUTPUT); // Define input/output pins
pinMode(echoPin4, INPUT); // Define input/output pins
/*pinMode(trigPin5, OUTPUT); // Define input/output pins
pinMode(echoPin5, INPUT); // Define input/output pins
pinMode(trigPin6, OUTPUT); // Define input/output pins
pinMode(echoPin6, INPUT); // Define input/output pins */
}

void loop() {
SonarSensor(trigPin1, echoPin1);
Sensor1 = distance;
SonarSensor(trigPin2, echoPin2);
Sensor2 = distance;
SonarSensor(trigPin3, echoPin3);
Sensor3 = distance;
SonarSensor(trigPin4, echoPin4);
Sensor4 = distance;
/*SonarSensor(trigPin5, echoPin5);
Sensor5 = distance;
SonarSensor(trigPin6, echoPin6);
Sensor6 = distance;*/

Serial.print(Sensor1);
Serial.print(" ");
Serial.print(Sensor2);
Serial.print(" ");
Serial.print(Sensor3);
Serial.print(" ");
Serial.println(Sensor4);
/*Serial.print(” – “);
Serial.print(Sensor5);
Serial.print(” – “);
Serial.println(Sensor6);*/

delay(200);

}

void SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;

}
