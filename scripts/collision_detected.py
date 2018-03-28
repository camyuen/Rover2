#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32

IR0=None
IR1=None
IR2=None
IR3=None
IR4=None
threshold=69


def fnc_callback0(range_msg0):
    global IR0
    IR0 =range_msg0.data
def fnc_callback1(range_msg1):
    global IR1
    IR1=range_msg1.data
def fnc_callback2(range_msg2):
    global IR2
    IR2 =range_msg2.data
def fnc_callback3(range_msg3):
    global IR3
    IR3 =range_msg3.data
def fnc_callback4(range_msg4):
    global IR4
    IR4 =range_msg4.data




if __name__=='__main__':
    rospy.init_node('collision_detected')

    sub=rospy.Subscriber('range0', Float32, fnc_callback0)
    sub=rospy.Subscriber('range1', Float32, fnc_callback1)
    sub=rospy.Subscriber('range2', Float32, fnc_callback2)
    sub=rospy.Subscriber('range3', Float32, fnc_callback3)
    sub=rospy.Subscriber('range4', Float32, fnc_callback4)

    pub=rospy.Publisher('collision_detected', Float32, queue_size=1)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
	global threshold
	threshold = 40
        if IR0<= threshold:
            collision_detected=1
        if IR1<= threshold:
            collision_detected=1
        if IR2<= threshold:
            collision_detected=1
	if IR3<= threshold:
	    collision_detected=1
	if IR4<= threshold:
            collision_detected=1
	else:
	    collision_detected=0
	pub.publish(collision_detected)
	rate.sleep()
