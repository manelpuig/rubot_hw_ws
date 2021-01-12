#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty


class ToggleLed:

	def __init__(self):
		self.chat_msg = ""

		self.toggle_msg=""

		self.sub = rospy.Subscriber("/chatter", String, self.chatterCb)

		self.pub = rospy.Publisher("/toggle_led", Empty, queue_size=10)


	def chatterCb(self, msg):
		self.chat_msg = msg.data
        #rospy.loginfo(rospy.get_caller_id() + ": I heard %s", msg.data)
        #if chat_msg=="Toggle LED!":
        toggle_msg=Empty() # message to be published
        rospy.loginfo("OK. I send Empty message to toggle LED")
        pub.publish(toggle_msg)
        #else:
        #    rospy.loginfo("NOT a correct order")

if __name__ == '__main__':
	rospy.init_node('rUBot_ctrl_node')
	ToggleLed()
	rospy.spin()
