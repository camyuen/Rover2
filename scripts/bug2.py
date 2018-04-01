#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn
from roverto.msg import bug2
import time


rospy.init_node('bug2', anonymous=True)
throttle_channel=3
steer_channel=4
collision_detected=0


def callback(data):
	global collision_detected
	collision_detected = data.data

def bug2main():
	pub=rospy.Publisher('rcout', bug2, queue_size=1)
	sub=rospy.Subscriber('collision_detected', Float32, callback) 
	rate=rospy.Rate(10)
	
	msg=bug2()
	msg.forward=0
	msg.reverse=0
	msg.spinright=0
	msg.spinleft=0
	msg.turnright=0
	msg.turnleft=0
	while not rospy.is_shutdown():

		if(collision_detected ==1):
		  rospy.wait_for_service('/mavros/set_mode')
		
		  try:
		    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		    mode='MANUAL'
		    resp1 = change_mode(0,mode)
		  except rospy.ServiceException as e:
		    print ("Service call failed: %s" %e)
		



		pub.publish(msg)


if __name__=='__main__':
	bug2main()	
