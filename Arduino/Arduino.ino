#include "RevCounter.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <donutdevice/Donut.h>
#include "MPU9250.h"
#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include "XL320.h"
#include "SoftwareSerial.h"
#include "TimerFour.h"

int curPos,goal;
XL320 steering;

int sampleFrequency;
float samplePeriod;
IntervalTimer dmTimer,pubTimer;
// These are general bounds for the steering servo and the
// Electronic Speed Controller (ESC)
const int minSteering = 422;
const int maxSteering = 600;
const int minThrottle = 1000 ;
const int maxThrottle = 2000 ;

Servo electronicSpeedController;  // The ESC works like a Servo

RevCounter RevCounter;
const float pi = 3.14159265359;

ros::NodeHandle_<ArduinoHardware, 6, 6, 4096*4, 4096*4> nh;
int pulsesRL, pulsesRR, pulsesFL, pulsesFR;

//initialize IMU message types and required strings
char base_link[] = "/base_link";
char odom[] = "/world";
char frame_id[] = "imu";
donutdevice::Donut donut_msg;
ros::Publisher donutpub("donut", &donut_msg);
ros::Time NowA, NowB;
void BuildMessages();


// an MPU9250 object with the MPU-9250 sensor on Teensy Chip Select pin 10
MPU9250 IMU(10);
float Aax, Aay, Aaz, Agx, Agy, Agz, Ahx, Ahy, Ahz, Bax, Bay, Baz, Bgx, Bgy, Bgz, Bhx, Bhy, Bhz;
int beginStatus;
volatile bool toggleData = false;
volatile bool useB = false;

//initialize variables for receiving drive messages and converting them to driving signals
//Callback for messages that steer the vehicle

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int escCommand, steeringAngle;
void driveCallback (const geometry_msgs::Twist&  twistMsg )
{
  steeringAngle = (int)fmap(twistMsg.angular.z, 1.0, -1.0, minSteering, maxSteering) ;

  // Check to make sure steeringAngle is within car range
  if (steeringAngle < minSteering) {
    steeringAngle = minSteering;
  }
  if (steeringAngle > maxSteering) {
    steeringAngle = maxSteering ;
  }


  escCommand = (int)fmap(twistMsg.linear.x, -20.0, 20.0, minThrottle, maxThrottle) ;

  // Check to make sure throttle command is within bounds
  if (escCommand < minThrottle) {
    escCommand = minThrottle;
  }
  if (escCommand > maxThrottle) {
    escCommand = maxThrottle ;
  }

  electronicSpeedController.writeMicroseconds(escCommand);
}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/cmd_vel", &driveCallback) ;

void IMUCallBack() {
  if (toggleData == false) {
    NowA = nh.now();
    IMU.getMotion9(&Aax, &Aay, &Aaz, &Agx, &Agy, &Agz, &Ahx, &Ahy, &Ahz);
    useB = false;
  }
  else {
    NowB = nh.now();
    IMU.getMotion9(&Bax, &Bay, &Baz, &Bgx, &Bgy, &Bgz, &Bhx, &Bhy, &Bhz);
    useB = true;
  }
}
void buildMessage() {
    if (useB) {
    toggleData = false;
    donut_msg.mpu.header.seq++;
    donut_msg.mpu.header.stamp = NowB;

    donut_msg.mpu.linear_acceleration.x = Bax;
    donut_msg.mpu.linear_acceleration.y = Bay;
    donut_msg.mpu.linear_acceleration.z = Baz;

    donut_msg.mpu.angular_velocity.x = Bgx;
    donut_msg.mpu.angular_velocity.y = Bgy;
    donut_msg.mpu.angular_velocity.z = Bgz;

    donut_msg.mpu.magnetic_field.x = Bhx;
    donut_msg.mpu.magnetic_field.y = Bhy;
    donut_msg.mpu.magnetic_field.z = Bhz;
    toggleData = true;
  }
  else {
    toggleData = true;
    donut_msg.mpu.header.seq++;
    donut_msg.mpu.header.stamp = NowA;

    donut_msg.mpu.linear_acceleration.x = Aax;
    donut_msg.mpu.linear_acceleration.y = Aay;
    donut_msg.mpu.linear_acceleration.z = Aaz;

    donut_msg.mpu.angular_velocity.x = Agx;
    donut_msg.mpu.angular_velocity.y = Agy;
    donut_msg.mpu.angular_velocity.z = Agz;

    donut_msg.mpu.magnetic_field.x = Ahx;
    donut_msg.mpu.magnetic_field.y = Ahy;
    donut_msg.mpu.magnetic_field.z = Ahz;
    toggleData = false;
    donut_msg.wheels.header.seq++;
    donut_msg.wheels.header.stamp = nh.now();

    cli();
    pulsesRR = RevCounter.readCounter(0);
    pulsesRL = RevCounter.readCounter(1);
    pulsesFR = RevCounter.readCounter(2);
    pulsesFL = RevCounter.readCounter(3);
    sei();

    donut_msg.wheels.rr = pulsesRR*pi;
    donut_msg.wheels.rl = pulsesRL*pi;
    donut_msg.wheels.fr = pulsesFR*pi;
    donut_msg.wheels.fl = pulsesFL*pi;
  }
}
void buildSMessage(){
  donut_msg.dynamixel.header.seq++;
  donut_msg.dynamixel.header.stamp = nh.now();
  donut_msg.dynamixel.angle = steering.GetValue(1,XL320::Address::PRESENT_POSITION);
  donut_msg.dynamixel.load = steering.GetValue(1,XL320::Address::PRESENT_LOAD);
}


void publishCallback() {
  buildMessage();

  donutpub.publish(&donut_msg);
}

void initMessages() {
  donut_msg.dynamixel.header.seq = 0;
  donut_msg.wheels.header.seq = 0;

  donut_msg.mpu.header.seq = 0;
  donut_msg.mpu.header.frame_id = frame_id;
}

volatile int dm_flag;
int dm_rate;
void dmCom(){
  steering.Write(1,XL320::Address::GOAL_POSITION,steeringAngle);
  buildSMessage();
}


void getRate(){
  sampleFrequency=0;
  if (nh.getParam("rate", &sampleFrequency,1) == false) {
    //default sample frequency
    sampleFrequency = 120;
    nh.loginfo("Rate not set, automatically set to 120Hz");
  }
  if (sampleFrequency > 1000){
    sampleFrequency = 1000;
    nh.loginfo("Teensy is capped at 1000hz, 1000hz is set");
  }
  nh.loginfo("ROS parameters set");
}

void setup(){
  nh.initNode();
  nh.advertise(donutpub);
  nh.subscribe(driveSubscriber) ;
  while(!nh.connected()) {nh.spinOnce();}
  nh.loginfo("Connection established");

  getRate();

  RevCounter.begin();


  // start communication with IMU and
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  IMU.setFilt(DLPF_BANDWIDTH_41HZ,9);
  pinMode(2,INPUT);
  attachInterrupt(2,IMUCallBack,RISING);
  nh.loginfo("IMU calibrated");

  //Build all the required messages with required covariances and frame id's
  initMessages();
  nh.loginfo("Messages built");

  // Attach the servos to actual pins
  //init servo
  //SerialUart3.begin(9600);
  //SerialUart3.begin(115200);
  Serial3.begin(1000000);
  steering.Begin(Serial3);
  electronicSpeedController.attach(9); // ESC is on pin 10
  // Initialize Steering and ESC setting
  steeringAngle = 511;
  electronicSpeedController.writeMicroseconds(1500) ;
  nh.loginfo("vehicle control started");
  delay(1000);

  dmTimer.priority(255);
  pubTimer.priority(140);
  dmTimer.begin(dmCom,100000);
  pubTimer.begin(publishCallback,(int)(1000000/sampleFrequency));

  nh.loginfo("Publish timer initialized");
}

void loop(){

  nh.spinOnce();
  if(!nh.connected()){
    electronicSpeedController.write(1490);
    dmTimer.end();
    pubTimer.end();
    nh.initNode();
    while(!nh.connected()) {nh.spinOnce();}
    nh.loginfo("Connection established");
    getRate();
    dmTimer.begin(dmCom,100000);
    pubTimer.begin(publishCallback,(int)(1000000/sampleFrequency));
    nh.loginfo("Publish timer Reset");
  }
}
