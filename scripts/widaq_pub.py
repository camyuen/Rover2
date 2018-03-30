#!/usr/bin/env python
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from roverto.msg import widaq
import rospy
import pexpect
import time
import os
import math
orientation_x=32
orientation_y=32
orientation_z=32
orientation_w=32
odom=69
def pulldata():
    import pexpect
    import time
    DEVICE = "54:6C:0E:B2:FA:84"

    print("Digiscan address:"),
    print(DEVICE)

    # Run gatttool interactively.
    print("Run gatttool...")
    child = pexpect.spawn("gatttool -I")
    child.sendline("sudo systemtl restart bluetooth")
    child.expect('.*')
    child.sendline("sudo hciconfig hci0 up")
    child.expect('.*')
    time.sleep(1)
# Connect to the device.
    print("Connecting to "),
    print(DEVICE),
    time.sleep(0.3)
    child.sendline("connect {0}".format(DEVICE))
    time.sleep(0.3)
    child.expect("Connection successful", timeout=10)
    print(" Connected!")
    # Digiscan

    child.sendline("char-write-cmd 27 01")      #enable sensors
    time.sleep(0.5)
    child.expect("", timeout=None)
    time.sleep(0.5)
    child.sendline("char-read-hnd 24")            
    time.sleep(1)
    child.expect("Characteristic value/descriptor: ", timeout=None)
    print("fuck")
    #if (child.before == ):
    child.expect("\r\n", timeout=None)

    data = [float(hexStrToInt(child.before[0:5])-198), float(hexStrToInt(child.before[12:17])-198), float(hexStrToInt(child.before[24:29])-198), float(hexStrToInt(child.before[36:41])-198)] 
    print val
    print("got past data")    
    #child.sendline("disconnect")
    #child.expect("\r\n", timeout=None)
    #child.expect(" Invalid file descriptor.\r\n", timeout=None)
    #child.expect("\r\n", timeout=None)
    #child.sendline("exit")
    #child.expect("\r\n", timeout=None)
    print data                                   
    return data

def hexStrToInt(hexstr):
	global val
	val = int(hexstr[0:2],16) - int(hexstr[3:5],16)
        if ((val&0x8000)==0x8000): # treat signed 16bits
        	val = -((val^0xffff)+1)
        return val


def callback0(data):
	rospy.loginfo(rospy.get_caller_id() +"\nposition:\nx: [{}]\ny: [{}]\nz: [{}]". 
        format(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z))
	positionx = data.pose.pose.position.x
	positiony = data.pose.pose.position.y
	odom.pose.pose.position.x = positionx 
 	odom.pose.pose.position.y = positiony  

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "\norientation:\nx: [{}]\ny: [{}]\nz: [{}]\nw: [{}]".
	format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
	global orientation_x
	global orientation_y
	global orientation_z
	global orientation_w
	orientation_x = float(data.orientation.x)
	orientation_y = float(data.orientation.y)
	orientation_z = float(data.orientation.z)
	orientation_w = float(data.orientation.w)

def yaw_calc():
	global t3
	global t4
	t3 = 2.0 * (orientation_w * orientation_z + orientation_x * orientation_y)
	t4 = 1.0 - 2.0 * (orientation_y * orientation_y + orientation_z * orientation_z)
	yaw = math.degrees(math.atan2(t3, t4))
	return yaw

def widaq_publish():
	
	pub=rospy.Publisher('widaq_data', widaq, queue_size=1)
	pub1=rospy.Publisher('toggle_widaq', Empty, queue_size=1)
	pub2=rospy.Publisher('widaq_data_odometry', Odometry, queue_size=1)   
	rospy.init_node('widaq_data')
	rospy.Subscriber("/mavros/imu/data", Imu, callback)
	rospy.Subscriber("/mavros/global_position/local",Odometry ,callback0)
	rate=rospy.Rate(10)
	msg=widaq()
	odom= Odometry()
	while not rospy.is_shutdown():
		global data
		global odom
		global widaq0
                global widaq1
                global widaq2
                global widaq3
		msg.yaw = yaw_calc()
		odom.pose.pose.position.z=yaw_calc()
		time.sleep(5)
		#pub1.publish()
		#time.sleep(1)
		msg.yaw = yaw_calc()
		pub1.publish()
		time.sleep(1)
		data = pulldata()
		msg.widaq0 = float(data[0])
		msg.widaq1 = float(data[1])
		msg.widaq2 = float(data[2])
		msg.widaq3 = float(data[3])
		odom.twist.twist.linear.x = float(data[0])
		odom.twist.twist.linear.y = float(data[1])
		odom.twist.twist.linear.z = float(data[2])
		odom.twist.twist.angular.x = float(data[3])   
		pub.publish(msg)
		pub2.publish(odom)


if __name__=='__main__':
	widaq_publish()

