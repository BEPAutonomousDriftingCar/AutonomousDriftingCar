
#include "XL320.h"
#include "SoftwareSerial.h"

int curPos,goal1, goal2;

SoftwareSerial SerialUart3(7, 8); // (RX, TX)

  XL320 steering;

  void Init()
  {

    //init servo
    //SerialUart3.begin(9600);
    //SerialUart3.begin(115200);
    SerialUart3.begin(1000000);
    steering.Begin(SerialUart3);

    //configure Serial2 for servo
    UART2_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
    CORE_PIN8_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
    
  }

void setup() {
    goal1 = 422;
    goal2 = 600;
    pinMode(13,OUTPUT);
    digitalWrite(13,HIGH);
    Serial.begin(9600);
    delay(2000);
    Serial.println("Serial has begun");
    Init();
    steering.Write(1,XL320::Address::BAUD_RATE,3);
    delay(100);
    Serial.println(steering.GetValue(1,XL320::Address::BAUD_RATE)); 
}

void loop() {
    steering.Write(1,XL320::Address::GOAL_POSITION,goal1);
    delay(2000);
    Serial.print("Left steering angle is ");
    Serial.println(steering.GetValue(1,XL320::Address::PRESENT_POSITION));
    Serial.print("Steering load is: ");
    Serial.println(steering.GetValue(1,XL320::Address::PRESENT_LOAD));  
    steering.Write(1,XL320::Address::GOAL_POSITION,goal2);
    delay(2000);
    Serial.print("Right steering angle is: ");
    Serial.println(steering.GetValue(1,XL320::Address::PRESENT_POSITION));
    Serial.print("Steering load is: ");
    Serial.println(steering.GetValue(1,XL320::Address::PRESENT_LOAD)-1024);  
}
