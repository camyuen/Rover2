#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn
from roverto.msg import bug2

rospy.init_node('MotorForward', anonymous=True)
throttle_channel=2
steer_channel=3
SPEED=1700 #need some tuning here, might need to use throttle and steer to get this bitch to turn a bit
throttle=0
def talker():
	pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
	rospy.Subscriber("rcout",bug2 ,callback)
	
	r = rospy.Rate(10) #10hzx
	while not rospy.is_shutdown():
		if (throttle==1):
			msg = OverrideRCIn()
			msg.channels[steer_channel]=SPEED
			rospy.loginfo(msg)
			pub.publish(msg)

def callback(data):
	global throttle
	throttle=data.forward





if __name__ == '__main__':
	talker()
