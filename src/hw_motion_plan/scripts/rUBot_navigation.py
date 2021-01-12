#!/usr/bin/env python

import rospy

from std_msgs.msg import String, Int32, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

# Initialise variables
range_msg=0
imu_msg=""
odom_msg=0

pub = None

#define function/functions to provide the required functionality
def rangeCb(r_msg): # msg could be any name
    global range_msg
    range_msg=r_msg.data
    rospy.loginfo(rospy.get_caller_id() + ": I heard %s", range_msg)
    vel_msg=Twist() # message to be published
    #linear_x = 0
    #angular_z = 0
    if range_msg<7:
        linear_x=0
        angular_z=1
        vel_msg.linear.x=linear_x
        vel_msg.angular.z=angular_z
        pub.publish(vel_msg)
        rospy.loginfo("Too close!. I send velocity message to /cmd_vel")
        
    else:
        #vel_msg.linear.x=1
        #vel_msg.angular.z=0
        rospy.loginfo("Far enough!")

def imuCb(i_msg):
    global imu_msg
    imu_msg=i_msg.data
    rospy.loginfo(rospy.get_caller_id() + ": I heard %s", imu_msg)

def odomCb(o_msg):
    global odom_msg
    odom_msg=o_msg.data
    rospy.loginfo(rospy.get_caller_id() + ": I heard %s", odom_msg)
    
if __name__=='__main__':

    try:
        #Add here the name of the ROS node. Node names are unique named. Here without "anonimous"!
        rospy.init_node('rUBot_ctrl_node')
        #subscribe to a topic using rospy.Subscriber class
        sub_range=rospy.Subscriber('/range', Int32, rangeCb)
        sub_imu=rospy.Subscriber('/imu', String, imuCb)
        sub_odom=rospy.Subscriber('/odom', Int32, odomCb)
        #publish messages to a topic using rospy.Publisher class
        pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  
        rospy.spin()
      
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")