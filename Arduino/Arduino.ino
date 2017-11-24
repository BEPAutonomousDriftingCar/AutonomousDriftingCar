//ifndef USE_TEENSY_HW_SERIAL
//#define USE_TEENSY_HW_SERIAL


#include "RevCounter.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

RevCounter RevCounter;
const float pi = 3.14159265359;


ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";
char frame_id[] = "imu";
sensor_msgs::Imu imu_msg;
ros::Publisher pub("imu/data_raw", &imu_msg);


void setup(){  
  nh.initNode();
  nh.advertise(pub);
   broadcaster.init(nh);
   RevCounter.begin();

  imu_msg.header.seq = 0;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = frame_id;

  imu_msg.orientation_covariance[0] = 0;
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
  imu_msg.header.seq++;
  imu_msg.header.stamp = nh.now();

  imu_msg.orientation.x = 0;
  imu_msg.orientation.y = 0;
  imu_msg.orientation.z = 0;
  imu_msg.orientation.w = 0;

  imu_msg.linear_acceleration.x = 0;
  imu_msg.linear_acceleration.y = 0;
  imu_msg.linear_acceleration.z = 0;

  imu_msg.angular_velocity.x = 0;
  imu_msg.angular_velocity.y = 0;
  imu_msg.angular_velocity.z = 0;


  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  t.transform.rotation.x = RevCounter.readCounter(0)*pi;
  t.transform.rotation.y = RevCounter.readCounter(1)*pi; 
  t.transform.rotation.z = RevCounter.readCounter(2)*pi; 
  t.transform.rotation.w = RevCounter.readCounter(3)*pi;  
  
  t.header.stamp = nh.now();
  
  pub.publish(&imu_msg);
  broadcaster.sendTransform(t);
  nh.spinOnce();

  delay(10);
}
