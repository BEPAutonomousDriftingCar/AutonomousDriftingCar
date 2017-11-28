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

// These are general bounds for the steering servo and the
// TRAXXAS Electronic Speed Controller (ESC)
const int minSteering = 55 ;
const int maxSteering = 125 ;
const int minThrottle = 1000 ;
const int maxThrottle = 2000 ;

Servo steeringServo;
Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo


RevCounter RevCounter;
const float pi = 3.14159265359;

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
ros::Publisher wheels("wheels",&t);

char base_link[] = "/base_link";
char odom[] = "/odom";
char frame_id[] = "imu";
double lastTime, nowTime;
double samplePeriod = 0.010;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
ros::Publisher pub("imu/data_raw", &imu_msg);
ros::Publisher magpub("imu/mag", &mag_msg);

void driveCallback ( const geometry_msgs::Twist&  twistMsg )
{
  
  int steeringAngle = fmap(twistMsg.angular.z, -1.0, 1.0, minSteering, maxSteering) ;
  // The following could be useful for debugging
  // str_msg.data= steeringAngle ;
  // chatter.publish(&str_msg);
  // Check to make sure steeringAngle is within car range
  if (steeringAngle < minSteering) { 
    steeringAngle = minSteering;
  }
  if (steeringAngle > maxSteering) {
    steeringAngle = maxSteering ;
  }
  steeringServo.write(steeringAngle) ;
  
  // ESC forward is between 0.5 and 1.0
  int escCommand ;
  //if (twistMsg.linear.x >= 0.5) {
  escCommand = (int)fmap(twistMsg.linear.x, -20.0, 20.0, minThrottle, maxThrottle) ;
  //} else {
  //  escCommand = (int)fmap(twistMsg.linear.x, 0.0, 1.0, 0.0, 180.0) ;
  //}
  // Check to make sure throttle command is within bounds
  if (escCommand < minThrottle) { 
    escCommand = minThrottle;
  }
  if (escCommand > maxThrottle) {
    escCommand = maxThrottle ;
  }
  // The following could be useful for debugging
  // str_msg.data= escCommand ;
  // chatter.publish(&str_msg);
  electronicSpeedController.writeMicroseconds(escCommand);
} 

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/cmd_vel", &driveCallback) ;

// an MPU9250 object with the MPU-9250 sensor on Teensy Chip Select pin 10
MPU9250 IMU(10);
float ax, ay, az, gx, gy, gz, hx, hy, hz;
int beginStatus;


void setup(){
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(magpub);
  nh.advertise(wheels);
  
  RevCounter.begin();

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);

  mag_msg.header.seq = 0;
  mag_msg.header.stamp = nh.now();
  mag_msg.header.frame_id = frame_id;

  mag_msg.magnetic_field_covariance[0] = 0;
  mag_msg.magnetic_field_covariance[1] = 0;
  mag_msg.magnetic_field_covariance[2] = 0;


  mag_msg.magnetic_field_covariance[3] = 0;
  mag_msg.magnetic_field_covariance[4] = 0;
  mag_msg.magnetic_field_covariance[5] = 0;

  mag_msg.magnetic_field_covariance[6] = 0;
  mag_msg.magnetic_field_covariance[7] = 0;
  mag_msg.magnetic_field_covariance[8] = 0;


  imu_msg.header.seq = 0;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = frame_id;

  imu_msg.orientation_covariance[0] = -1;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;

  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0;
  imu_msg.orientation_covariance[5] = 0;

  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0;

  imu_msg.angular_velocity_covariance[0] = 0;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;

  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0;
  imu_msg.angular_velocity_covariance[5] = 0;

  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0;

  imu_msg.linear_acceleration_covariance[0] = 0;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;

  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0;
  imu_msg.linear_acceleration_covariance[5] = 0;

  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0;

  nh.subscribe(driveSubscriber) ;
  // Attach the servos to actual pins
  steeringServo.attach(9); // Steering servo is attached to pin 9
  electronicSpeedController.attach(8); // ESC is on pin 10
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  steeringServo.write(90) ;
  electronicSpeedController.writeMicroseconds(1500) ;
  delay(1000);
  lastTime = nh.now().toSec();
}

void loop(){
  nowTime=nh.now().toSec();
  if(nowTime-lastTime >= samplePeriod){
    IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
  
    imu_msg.header.seq++;
    imu_msg.header.stamp = nh.now();

    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 0;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    mag_msg.header.seq++;
    mag_msg.header.stamp = nh.now();
    
    mag_msg.magnetic_field.x = hx;
    mag_msg.magnetic_field.y = hy;
    mag_msg.magnetic_field.z = hz;

    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.rotation.x = RevCounter.readCounter(0)*pi;
    t.transform.rotation.y = RevCounter.readCounter(1)*pi; 
    t.transform.rotation.z = RevCounter.readCounter(2)*pi; 
    t.transform.rotation.w = RevCounter.readCounter(3)*pi;  
    
    t.header.stamp = nh.now();
  
    pub.publish(&imu_msg);
    magpub.publish(&mag_msg);
    wheels.publish(&t);
    lastTime=nh.now().toSec();
  }
  if(nh.connected() == false){
    electronicSpeedController.write(1490);
  }
  nh.spinOnce();
  delay(1);
}


// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


