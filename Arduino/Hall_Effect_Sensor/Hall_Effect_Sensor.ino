 volatile byte revolutions;
 volatile byte turns;
 float turnspeed; //amount of turns per second
 unsigned long timeold;
 int velocity; //in cm/second
 int distance; // in cm
 float turntime; // time for one turn
 int circumference;


////////// FILL IN ////////////
 int diameter = 15; // Outside tire diameter in centimetres
 int magnetamount = 1; // Amount of magnets per wheel
////////// FILL IN ////////////

 void setup() {
   Serial.begin(9600);
   attachInterrupt(0, magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)
   revolutions = 0;
   turnspeed = 0;
   timeold = 0;
   turns = 0;
   
 }
 void loop()//Measure RPM
 {
   circumference = diameter*3.14;
   
   if (revolutions >= (10*magnetamount)) { 
     turntime = (millis()-timeold)/(revolutions/magnetamount);
     turnspeed = 1000/turntime;
     Serial.print("Runtime = ");
     Serial.println(millis()-timeold);   
     timeold = millis();
     distance = (turns * circumference); // distance of car in cm
     velocity = (turnspeed * circumference); // speed of car in cm/s  
     Serial.print("Turntime = ");
     Serial.print(turntime);
     Serial.print(" Turnspeed = ");
     Serial.println(turnspeed);
     Serial.print("Distance = ");
     Serial.println(distance);
     Serial.print("Velocity = ");
     Serial.println(velocity);
     
     revolutions = 0;


   }
 }
 void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
   revolutions++;
   turns++;
   Serial.print("Revolutions in set: ");
   Serial.print(revolutions);
   Serial.print(" Total revolutions: ");
   Serial.println(turns);
 }
