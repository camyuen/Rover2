#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn
from roverto.msg import bug2
import time


rospy.init_node('bug2', anonymous=True)
throttle_channel=3
steer_channel=4

def bug2main():
	pub=rospy.Publisher('rcout', bug2, queue_size=1)
	rate=rospy.Rate(10)

	msg=bug2()
	msg.forward=1
	msg.reverse=0
	msg.spinright=0
	msg.spinleft=0
	msg.turnright=0
	msg.turnleft=0
	while not rospy.is_shutdown():
		pub.publish(msg)


if __name__=='__main__':
	bug2main()	
