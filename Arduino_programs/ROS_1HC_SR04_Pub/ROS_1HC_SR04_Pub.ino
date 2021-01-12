/*
 * ROS Ultrasonic Simple
*/

#include <Ultrasonic.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

Ultrasonic ultrasonic(12, 13);
int distance;

ros::NodeHandle nh;

sensor_msgs::Range range_msg;

ros::Publisher pub_range_ultrasound("/ultrasound", &range_msg);

void setup() {

   nh.initNode();
   nh.advertise(pub_range_ultrasound);
}

void loop() {
    
    /*distance = ultrasonic.read();
  
    Serial.print("Distance in CM: ");
    Serial.println(distance);
    delay(100);*/

    range_msg.range = ultrasonic.read();
    pub_range_ultrasound.publish(&range_msg);

    nh.spinOnce();
}