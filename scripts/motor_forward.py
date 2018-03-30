#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn

rospy.init_node('MotorForward', anonymous=True)
throttle_channel=3
steer_channel=4
SPEED=1700 #pwm for forward
def talker():
	pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
	rospy.Subscriber("rcout",motorrcout ,callback)
	
	r = rospy.Rate(10) #10hzx
	
	if (throttle==1):
		msg = OverrideRCIn()
		msg.channels[throttle_channel]=SPEED
		rospy.loginfo(msg)
		pub.publish(msg)

def callback(data):
	global throttle
	throttle=data.motorrcout.forward





if __name__ == '__main__':
	talker()
