/*
 * rosserial sensors range, imu, ultrasound to motor
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>


#define EN_L 9
#define IN1_L 10
#define IN2_L 11

#define EN_R 8
#define IN1_R 12
#define IN2_R 13


double w_r=0, w_l=0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.295;


ros::NodeHandle  nh;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;


void velCb(const geometry_msgs::Twist& vel_msg){
  speed_ang = vel_msg.angular.z;
  speed_lin = vel_msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", velCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

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
  Motors_init();
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
  
  MotorL(w_l*10);
  MotorR(w_r*10);
  nh.spinOnce();
  //delay(1); //low value (or eliminated) to avoid unsync!
}


void Motors_init(){

 pinMode(EN_L, OUTPUT);

 pinMode(EN_R, OUTPUT);

 pinMode(IN1_L, OUTPUT);

 pinMode(IN2_L, OUTPUT);

 pinMode(IN1_R, OUTPUT);

 pinMode(IN2_R, OUTPUT);

 digitalWrite(EN_L, LOW);

 digitalWrite(EN_R, LOW);

 digitalWrite(IN1_L, LOW);

 digitalWrite(IN2_L, LOW);

 digitalWrite(IN1_R, LOW);

 digitalWrite(IN2_R, LOW);

}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){

     analogWrite(EN_L, Pulse_Width1);

     digitalWrite(IN1_L, HIGH);

     digitalWrite(IN2_L, LOW);

 }

 if (Pulse_Width1 < 0){

     Pulse_Width1=abs(Pulse_Width1);

     analogWrite(EN_L, Pulse_Width1);

     digitalWrite(IN1_L, LOW);

     digitalWrite(IN2_L, HIGH);

 }

 if (Pulse_Width1 == 0){

     analogWrite(EN_L, Pulse_Width1);

     digitalWrite(IN1_L, LOW);

     digitalWrite(IN2_L, LOW);

 }

}


void MotorR(int Pulse_Width2){


 if (Pulse_Width2 > 0){

     analogWrite(EN_R, Pulse_Width2);

     digitalWrite(IN1_R, LOW);

     digitalWrite(IN2_R, HIGH);

 }

 if (Pulse_Width2 < 0){

     Pulse_Width2=abs(Pulse_Width2);

     analogWrite(EN_R, Pulse_Width2);

     digitalWrite(IN1_R, HIGH);

     digitalWrite(IN2_R, LOW);

 }

 if (Pulse_Width2 == 0){

     analogWrite(EN_R, Pulse_Width2);

     digitalWrite(IN1_R, LOW);

     digitalWrite(IN2_R, LOW);

 }

}
