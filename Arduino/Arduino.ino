#include "RevCounter.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "MPU9250.h"

RevCounter RevCounter;
const float pi = 3.14159265359;

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";
char frame_id[] = "imu";
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField magneticfield_msg;
ros::Publisher pub("imu/data_raw", &imu_msg);
ros::Publisher pub2("imu/mag", &magneticfield_msg);


// an MPU9250 object with the MPU-9250 sensor on Teensy Chip Select pin 10
MPU9250 IMU(10);
float ax, ay, az, gx, gy, gz, hx, hy, hz;
int beginStatus;


void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);  
  nh.initNode();
  nh.advertise(pub);
  broadcaster.init(nh);
  
  RevCounter.begin();

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);

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
}

void loop(){
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

  magneticfield_msg.magnetic_field.x = hx;
  magneticfield_msg.magnetic_field.y = hy;
  magneticfield_msg.magnetic_field.z = hz;

  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  t.transform.rotation.x = RevCounter.readCounter(0)*pi;
  t.transform.rotation.y = RevCounter.readCounter(1)*pi; 
  t.transform.rotation.z = RevCounter.readCounter(2)*pi; 
  t.transform.rotation.w = RevCounter.readCounter(3)*pi;  
  
  t.header.stamp = nh.now();
  
  pub.publish(&imu_msg);
  pub2.publish(&magneticfield_msg);
  broadcaster.sendTransform(t);
  nh.spinOnce();

  delay(10);
}
