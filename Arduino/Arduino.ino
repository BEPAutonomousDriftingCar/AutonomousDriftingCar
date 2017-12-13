#include "RevCounter.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
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
geometry_msgs::QuaternionStamped w;
geometry_msgs::TransformStamped RL, RR, FL, FR;
geometry_msgs::QuaternionStamped s;
int pulsesRL, pulsesRR, pulsesFL, pulsesFR;
ros::Publisher wheels("wheels",&w);
ros::Publisher steer("steer",&s);
tf::TransformBroadcaster broadcaster;

//initialize IMU message types and required strings
char base_link[] = "/base_link";
char odom[] = "/world";
char frame_id[] = "imu";
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
ros::Publisher imupub("imu/data_raw", &imu_msg);
ros::Publisher magpub("imu/mag", &mag_msg);
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
void buildIMUMessage() {
    if (useB) {
    toggleData = false;
    imu_msg.header.seq++;
    imu_msg.header.stamp = NowB;
    
    imu_msg.linear_acceleration.x = Bax;
    imu_msg.linear_acceleration.y = Bay;
    imu_msg.linear_acceleration.z = Baz;

    imu_msg.angular_velocity.x = Bgx;
    imu_msg.angular_velocity.y = Bgy;
    imu_msg.angular_velocity.z = Bgz;

    mag_msg.header.seq++;
    mag_msg.header.stamp = imu_msg.header.stamp;
    
    mag_msg.magnetic_field.x = Bhx;
    mag_msg.magnetic_field.y = Bhy;
    mag_msg.magnetic_field.z = Bhz;
    toggleData = true;
  }
  else {
    toggleData = true;
    imu_msg.header.seq++;
    imu_msg.header.stamp = NowA;
    
    imu_msg.linear_acceleration.x = Aax;
    imu_msg.linear_acceleration.y = Aay;
    imu_msg.linear_acceleration.z = Aaz;

    imu_msg.angular_velocity.x = Agx;
    imu_msg.angular_velocity.y = Agy;
    imu_msg.angular_velocity.z = Agz;

    mag_msg.header.seq++;
    mag_msg.header.stamp = imu_msg.header.stamp;
    
    mag_msg.magnetic_field.x = Ahx;
    mag_msg.magnetic_field.y = Ahy;
    mag_msg.magnetic_field.z = Ahz;
    toggleData = false;
  }
}
void buildSMessage(){
  s.header.seq++;
  s.header.stamp = nh.now();
  s.quaternion.w = steering.GetValue(1,XL320::Address::PRESENT_POSITION);
  s.quaternion.x = steering.GetValue(1,XL320::Address::PRESENT_LOAD);
}
void buildWMessage() {
  w.header.seq++;
  w.header.stamp = nh.now();

  cli();
  pulsesRR = RevCounter.readCounter(0);
  pulsesRL = RevCounter.readCounter(1);
  pulsesFR = RevCounter.readCounter(2); 
  pulsesFL = RevCounter.readCounter(3);
  sei();

  w.quaternion.x = pulsesRR*pi;
  w.quaternion.y = pulsesRL*pi; 
  w.quaternion.z = pulsesFR*pi; 
  w.quaternion.w = pulsesFL*pi;  
}
  
void publishCallback() {
  buildIMUMessage();
  buildWMessage(); 

  steer.publish(&s);
  wheels.publish(&w);
  imupub.publish(&imu_msg);
  magpub.publish(&mag_msg);
}

void initMessages() {
  s.header.seq = 0;
  w.header.seq = 0;

  mag_msg.header.seq = 0;
  mag_msg.header.frame_id = frame_id;

  imu_msg.header.seq = 0;
  imu_msg.header.frame_id = frame_id;

  imu_msg.orientation_covariance[0] = -1;
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
  nh.advertise(imupub);
  nh.advertise(magpub);
  nh.advertise(wheels);
  nh.advertise(steer);
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



