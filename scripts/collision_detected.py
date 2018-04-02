#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn
from roverto.msg import ir_array

IR0
IR1
IR2
IR3
IR4
BUMP0
BUMP1

threshold1=30
threshold2=40
threshold3=30
collision_detected=0
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
def fnc_callback5(bump_msg0):
    global BUMP0
    BUMP1 =bump_msg0.data

def fnc_callback4(bump_msg1):
    global BUMP1
    BUMP1 =bump_msg1.data




if __name__=='__main__':
    rospy.init_node('collision_detected')

    sub0=rospy.Subscriber('range0', Float32, fnc_callback0)
    sub1=rospy.Subscriber('range1', Float32, fnc_callback1)
    sub2=rospy.Subscriber('range2', Float32, fnc_callback2)
    sub3=rospy.Subscriber('range3', Float32, fnc_callback3)
    sub4=rospy.Subscriber('range4', Float32, fnc_callback4)
    sub5=rospy.Subscriber('bump0', Float32, fnc_callback5)
    sub6=rospy.Subscriber('bump1', Float32, fnc_callback6)



    pub=rospy.Publisher('collision_detected', Float32, queue_size=1)
    pub1=rospy.Publisher('ir_array', ir_array, queue_size=1)
    rate=rospy.Rate(10)
    msg = ir_array()
    while not rospy.is_shutdown():
	global threshold
	threshold = 40
	

	print ("\nrangesensors:\n0: [{}]\n1: [{}]\n2: [{}]\n3: [{}]\n4: [{}]".
	format(IR0, IR1, IR2, IR3, IR4))
        msg.IR0=IR0
	msg.IR1=IR1
	msg.IR2=IR2
        msg.IR3=IR3
	msg.IR4=IR4
	msg.BUMP0=BUMP0
	msg.BUMP1=BUMP1    
	while (IR2<= threshold2 or IR1<=threshold1 or IR3<=threshold3 or BUMP0==1 or BUMP1==1):
		collision_detected = 1
		pub.publish(collision_detected)
	else: 
		collision_detected=0
		pub.publish(collision_detected)

	pub1.publish(msg)
        
	#pub.publish(collision_detected)

	#print ("\nrangesensors:\n0: [{}]\n1: [{}]\n2: [{}]\n3: [{}]\n4: [{}]ncollision: {[]}".
	#format(IR0, IR1, IR2, IR3, IR4, collision_detected))




#	if collision_detected=1:
#		rospy.wait_for_service('/mavros/set_mode')
#		change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
#		resp1 = change_mode(custom_mode='manual')
	rate.sleep()
