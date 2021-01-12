/*
 * rosserial sensors range, imu, ultrasound to motor
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;


void velCb(const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> vel_sub("cmd_vel", velCb );

std_msgs::Int32 range_msg;
ros::Publisher range_pub("range", &range_msg);

int obstacle=6;

std_msgs::String imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

char rpy[]="R60P30Y30";

std_msgs::Int32 odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

int distance=10;

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(range_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.subscribe(vel_sub);
}

void loop()
{
  range_msg.data = obstacle;
  range_pub.publish( &range_msg );
  imu_msg.data = rpy;
  imu_pub.publish( &imu_msg );
  odom_msg.data = distance;
  odom_pub.publish( &odom_msg );
  nh.spinOnce();
  delay(1000);
}