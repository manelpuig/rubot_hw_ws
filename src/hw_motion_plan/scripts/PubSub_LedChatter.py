#!/usr/bin/env python
# the previous line has allways to be the first line
#remove or add the library/libraries for ROS
import rospy

#remove or add the message type
from std_msgs.msg import String, Empty

# Initialise variables
chat_msg=""
pub = None

#define function/functions to provide the required functionality
def chatterCb(msg): # msg could be any name
    global chat_msg
    chat_msg=msg.data
    rospy.loginfo(rospy.get_caller_id() + ": I heard %s", chat_msg)  
    if chat_msg=="Toggle LED!":
        toggle_msg=Empty() # message to be published
        rospy.loginfo("OK. I send Empty message to toggle LED")
        pub.publish(toggle_msg)
    else:
        rospy.loginfo("NOT a correct order")


if __name__=='__main__':

    try:
        #Add here the name of the ROS node. Node names are unique named. Here without "anonimous"!
        rospy.init_node('rUBot_ctrl_node')
        #subscribe to a topic using rospy.Subscriber class
        sub=rospy.Subscriber('/chatter', String, chatterCb)
        #publish messages to a topic using rospy.Publisher class
        pub=rospy.Publisher('/toggle_led', Empty, queue_size=1)
  
        rospy.spin()
      
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")