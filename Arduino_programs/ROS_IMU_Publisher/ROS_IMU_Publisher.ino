#include <ros.h>
#include <std_msgs/String.h>
#include "MPU9250.h"

MPU9250 mpu;

//Set up the ros node and publisher
std_msgs::String imu_msg;
ros::Publisher imu("imu", &imu_msg);
ros::NodeHandle nh;


                           
void setup()
{

  nh.initNode();
  nh.advertise(imu);
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  mpu.setup();
}


void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16)
    {
        mpu.update();
        mpu.print();

        roll=mpu.getRoll();
        pitch=mpu.getPitch();
        yaw=mpu.getYaw();
        String R = String(roll);
        String P = String(pitch);
        String Y = String(string);
        String data = "R" + R + "P"+ P + "Y" + Y+ "F";
        Serial.println(data);
        int length = data.indexOf("F") +2;
        char data_final[length+1];

        imu_msg.data = data_final;
        imu.publish(&imu_msg);
        nh.spinOnce();

        prev_ms = millis();
    }  
  
}
