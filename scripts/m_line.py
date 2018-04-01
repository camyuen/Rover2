#!/usr/bin/env python
#this node calculates the m line 
import rospy
import os
import math
import numpy
import time
from mavros_msgs.msg import Waypoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
collision_detected=69
m_line_a=[420,69]
m_line_b=[8008,666]



def callback(data):
  global m_line_b
  rospy.loginfo(rospy.get_caller_id() + "\nWaypoint:\nx: [{}]\ny: [{}]\nz: [{}]".
  format(data.x_lat, data.y_long))
  m_line_b=[data.x_lat, data.y_long]

def callback1(data):
  global m_line_a
  rospy.loginfo(rospy.get_caller_id() + "\nCurrentLocation:\nx: [{}]\ny: [{}]\nz: [{}]".
  format(data.x_lat, data.y_long))
  m_line_a=[data.latitude, data.longitude]

def callback2(data):
  global collision_detected
  rospy.loginfo(rospy.get_caller_id() + "\nCollisionDetected:\nx: [{}]".
  format(data.collision_detected))
  collision_detected=data.collision_detected

def get_slope(a, b):
  return (b[1] - a[1]) / (b[0] - a[0])


def get_intercept(a, slope):
  return a[1] - (slope * a[0])


def on_m_line(a, m, b):
  y_value = m * a[0] + b
  close = numpy.isclose(y_value, a[1], rtol=1e-9, atol=0.0)
  if (close == 'TRUE'):
    return 1
  else:
    return 0
#isclose(a, b,  rel_tol=1e-09, abs_tol=0.0)
#if it is close enough

def mline():
  #print dir(math)
  rospy.Subscriber("/mavros/mission/waypoints",Waypoint, callback)
  rospy.Subscriber("/mavros/global_position/local", NavSatFix, callback1)
  rospy.Subscriber("/collision_detected", Float32, callback2)

  rospy.init_node('widaq_data')
  pub=rospy.Publisher('m_line_or_nah', Float32, queue_size=1)
  rate=rospy.Rate(10)

  reset=0



  while not rospy.is_shutdown():
    if (reset==0, collision_detected==1):
      slope=get_slope(m_line_a, m_line_b)
      intercept=get_intercept(m_line_a, slope)
      reset=1

    m_line=on_m_line(m_line_a, slope, intercept)

    if (m_line==0): 
      pub.publish(m_line)
    if (m_line==1):
      pub.publish(m_line)
      reset=0

if __name__=='__main__':
  mline()

