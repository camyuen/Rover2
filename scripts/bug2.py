#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float32
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import OverrideRCIn
from roverto.msg import bug2
from roverto.msg import ir_array
import time



sensor_fornt=415
sensor_right=415
sensor_left=415
sensor_right_corner=415
sensor_left_corner=415
bump_sensor_right=0
bump_sensor_left=0
collision_detected=0





sensor_front_detection = 69.0 # change values after testing to find thresholds
sensor_min_reading = 40.0 #(This says if the variable has a reading or not)
bump_sensor_trigger = 420.0 # change values (This says if the bump sensor is activated or not)

sensor_min_l = 69.0 # minimum threshold distance for rover to be away from wall in left wall following
sensor_min_r = 69.0 # change values after testing to find thresholds
sensor_max_l = 69.0 # change values after testing to find thresholds
sensor_max_r = 69.0 # change values after testing to find thresholds

dist_center_to_sensor = 12.0 # distance value from center of rover (center along left sensor axis not absolute) to side sensors
angle = 15.0 # turn angle for corrections to wall, can be adjusted here and code will adjust
tolerance_angle = 5.0 # angle of tolerance for wall following (rover will only correct if greater than this angle)




throttle_channel=3
steer_channel=4


def callback(data):
	global collision_detected
	collision_detected = data.data

def callback0(data):
  global sensor_left
  global sensor_left_corner
  global sensor_front
  global sensor_right_corner
  global sensor_right
  global bump_sensor_left
  global bump_sensor_right
  sensor_left=data.IR0
  sensor_left_corner=data.IR1
  sensor_front=data.IR2
  sensor_right_corner =data.IR3
  sensor_right=data.IR4
  bump_sensor_left=data.BUMP0
  bump_sensor_right=data.BUMP1


def bug2main():
	rospy.init_node('bug2')
	pub=rospy.Publisher('rcout', bug2, queue_size=1)
	sub=rospy.Subscriber('collision_detected', Float32, callback)
	sub1=rospy.Subscriber('ir_array', ir_array, callback0)
	rate=rospy.Rate(10)

	msg=bug2()
	msg.forward=0
	msg.reverse=0
	msg.spinright=0
	msg.spinleft=0
	msg.turnright=0
	msg.turnleft=0



	while not rospy.is_shutdown():###this is the in main function that will constantly run
		#this shit turns it to manuel mode when we hit something
		if collision_detected == 1:
                  set_mode_manual()
		for retry in range(10):
                  test = right_or_left()
		if test == "error":
   		  test = "right"
		if test == "right":
 		  turn_right_wall_detected()
		elif test == "left":
		  turn_left_wall_detected()
		motor_forward()
                time.sleep(3)
                motor_stop()
		main_code(test, 0)




def main_code(test, num):
    if num != 0:
        if test == "right":
            turn_right_wall_detected()
        elif test == "left":
            turn_left_wall_detected()
        go_straight_three_seconds() # Possibly adjust this
    if test == "right":
        wall_following_wall_on_left()
    elif test == "left":
        wall_following_wall_on_right()
    return





#********** FUNCTIONS FOR MAIN CODE **********

# Function: turn_right_wall_detected
# Purpose: Turns the rover right when a wall is first detected
# Parameters: N/A
def turn_right_wall_detected():
    motor_spin_right()
    while sensor_left >= sensor_min_reading:
        pass
    motor_stop()
    if sensor_left_corner >= sensor_min_reading:
        motor_spin_left()
        while sensor_left_corner < sensor_min_reading:
            pass
        motor_stop()
        motor_forward()
        while sensor_left >= sensor_min_reading:
            pass
        motor_stop()
    turn_to_parallel_left_initial()
    return

# Function: turn_left_wall_detected
# Purpose: Turns the rover left when a wall is first detected
# Parameters: N/A
def turn_left_wall_detected():
    motor_spin_left()
    while sensor_right >= sensor_min_reading:
        pass
    motor_stop()
    if sensor_right_corner >= sensor_min_reading:
        motor_spin_right()
        while sensor_right_corner < sensor_min_reading:
            pass
        motor_stop()
        motor_forward()
        while sensor_right >= sensor_min_reading:
            pass
        motor_stop()
    turn_to_parallel_right_initial()
    return

# Function: go_straight_three_seconds
# Purpose: Turn the motors straight for three seconds
# Parameters: N/A
def go_straight_three_seconds():
    check_trigger()
    motor_forward()
    time.sleep(3)
    motor_stop()
    check_trigger()
    return

# Function: right_or_left
# Purpose: Determines if a left or right turn should be made
# Parameters: N/A
# Returns: "right" if right turn required, "left" if left turn or "error" if no direction determined
def right_or_left():
    if sensor_left_corner <= sensor_right_corner:
        return "right"
    elif sensor_right_corner < sensor_left_corner:
        return "left"
    else:
        return "error"

def turn_to_parallel_left_initial():
    motor_stop()
    c_equiv = sensor_left_corner * math.cos(math.radians(30.0))
    if c_equiv > sensor_left:
        motor_spin_left()
        while sensor_left_corner * math.cos(math.radians(30.0)) > sensor_left:
            pass
    elif c_equiv < sensor_left:
        motor_spin_right()
        while sensor_left_corner * math.cos(math.radians(30.0)) < sensor_left:
            pass
    motor_stop()
    return

def turn_to_parallel_right_initial():
    motor_stop()
    c_equiv = sensor_right_corner * math.cos(math.radians(30.0))
    if c_equiv > sensor_right:
        motor_spin_right()
        while sensor_right_corner * math.cos(math.radians(30.0)) > sensor_right:
            pass
    elif c_equiv < sensor_right:
        motor_spin_left()
        while sensor_right_corner * math.cos(math.radians(30.0)) < sensor_right:
            pass
    motor_stop()
    return

def check_trigger():
    if m_line_trigger == 1:
        end_bug()
    return
# ********** WALL FOLLOWING ON LEFT **********

# Function: wall_following_wall_on_left
# Purpose: Follow a wall when the left side of the rover is facing the wall
# Parameters: N/A
def wall_following_wall_on_left():
    check_trigger()
    while sensor_front > sensor_front_detection and bump_sensor_right < bump_sensor_trigger and bump_sensor_left < bump_sensor_trigger:
        while sensor_left < sensor_min_reading and sensor_left_corner < sensor_min_reading: # aka the left and left corner sensor reads that there is a wall
            motor_forward()
            check_trigger()
            c_equiv = sensor_left_corner * math.cos(math.radians(30.0))
            if sensor_left > c_equiv:
                if sensor_left_corner < (sensor_left * math.cos(math.radians(tolerance_angle)) - 12.0 * math.sin(math.radians(tolerance_angle))) / math.cos(math.radians(30.0 - tolerance_angle)):
                    turn_to_parallel_leftwall()
            elif sensor_left < c_equiv:
                if sensor_left_corner > (sensor_left * math.cos(math.radians(tolerance_angle)) + 12.0 * math.sin(math.radians(tolerance_angle))) / math.cos(math.radians(30.0 + tolerance_angle)):
                    turn_to_parallel_leftwall()
            motor_forward()
            check_trigger()
            mix = sensor_left
            if sensor_min_l - dist_center_to_sensor < mix < sensor_max_l - dist_center_to_sensor:
                pass
            elif mix > sensor_max_l - dist_center_to_sensor:
                check_trigger()
                turn_to_parallel_leftwall()
                turn_left_towards_wall()
                go_straight_until_center_leftwall("toward")
                turn_to_parallel_leftwall()
            elif mix < sensor_min_l - dist_center_to_sensor:
                turn_to_parallel_leftwall()
                turn_right_away_from_wall()
                go_straight_until_center_leftwall("away")
                turn_to_parallel_leftwall()
        check_trigger()
        motor_stop()
        if sensor_left < sensor_min_reading < sensor_left_corner:
            l_dist_maintain = sensor_left
            go_straight_only_left_sensor(l_dist_maintain)
        go_straight_and_corner_left()
        go_straight_until_detection_left()
        wall_following_wall_on_left()
    check_trigger()
    motor_stop()
    if sensor_front < sensor_front_detection or bump_sensor_right > bump_sensor_trigger or bump_sensor_left > bump_sensor_trigger:
        if bump_sensor_right > bump_sensor_trigger or bump_sensor_left > bump_sensor_trigger: # Maybe change up code later to take in corner sensor data
            motor_reverse()
            time.sleep(1) # one second delay
            motor_stop()
        main_code("right", 69)
    else:
        wall_following_wall_on_left()
    return


def turn_to_parallel_leftwall():
    motor_stop()
    check_trigger()
    c_equiv = sensor_left_corner * math.cos(math.radians(30.0))
    if c_equiv > sensor_left:
        motor_spin_left()
        while sensor_left_corner * math.cos(math.radians(30.0)) > sensor_left:
            pass
    elif c_equiv < sensor_left:
        motor_spin_right()
        while sensor_left_corner * math.cos(math.radians(30.0)) < sensor_left:
            pass
    motor_stop()
    check_trigger()
    return


# Function: turn_left_towards_wall
# Purpose: When rover is parallel to wall in left wall following mode and is too far from the wall this will turn it towards the wall
# Parameters: N/A
# Notes: parameter angle can be changed at top of code
def turn_left_towards_wall():
    check_trigger()
    dist = sensor_left
    if dist < sensor_max_l - dist_center_to_sensor:
        return
    c_desired = (dist + dist_center_to_sensor - 12.0 * math.cos(math.radians(angle)) - 14.5 * math.sin(math.radians(angle))) / math.cos(math.radians(30.0 - angle))
    l_desired = (dist + dist_center_to_sensor - 12.0 * math.cos(math.radians(angle)) - 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
    motor_spin_left()
    while sensor_left < l_desired:
        check_trigger()
    if sensor_left > sensor_min_reading:
        motor_stop()
        motor_spin_left()
        while sensor_left_corner > c_desired:
            pass
    motor_stop()
    check_trigger()
    return



# Function: turn_right_away_from_wall
# Purpose: When rover is parallel to wall in left wall following mode and is too close to the wall this will turn it away from the wall
# Parameters: N/A
# Notes: parameter angle can be changed at top of code
def turn_right_away_from_wall():
    check_trigger()
    dist = sensor_left
    if dist > sensor_min_l:
        return
    l_desired = (dist + dist_center_to_sensor - 12.0 * math.cos(math.radians(angle)) + 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
    motor_spin_right()
    while sensor_left < l_desired:
        check_trigger()
    motor_stop()
    return



# Function: go_straight_until_center_leftwall
# Purpose: After correction turn this function will move the rover into the center of the tolerances
# Parameters: direction, either "toward" or "away"
# Notes: sensor_min_l and sensor_max_l can be changed at top of code
def go_straight_until_center_leftwall(direction):
    check_trigger()
    if direction == "toward":
        l_desired = (0.5 * (sensor_min_l + sensor_max_l) - 12.0 * math.cos(math.radians(angle)) - 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
        motor_forward()
        while sensor_left > l_desired:
            check_trigger()
    elif direction == "away":
        l_desired = (0.5 * (sensor_min_l + sensor_max_l) - 12.0 * math.cos(math.radians(angle)) + 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
        motor_forward()
        while sensor_left < l_desired:
            check_trigger()
    motor_stop()
    check_trigger()
    return



# Function: go_straight_only_left_sensor
# Purpose: Rover is near end of a wall and left corner sensor no longer detects so this program maintains left sensor distance and goes straight until wall cleared
# Parameters: dist, the distance of the left sensor when function called
def go_straight_only_left_sensor(dist):
    check_trigger()
    if sensor_left < sensor_min_reading:
        motor_forward()
        while dist - 2 < sensor_left < dist + 2:
            check_trigger()
        if dist + 2 < sensor_left < sensor_min_reading:
            motor_stop()
            motor_spin_left()
            time.sleep(1)
            motor_stop()
            go_straight_only_left_sensor(dist)
        elif sensor_left < dist - 2:
            motor_stop()
            motor_spin_right()
            time.sleep(1)
            motor_stop()
            go_straight_only_left_sensor(dist)
    motor_stop()
    check_trigger()
    return

# Function: go_straight_and_corner_left
# Purpose: Left sensor clears the wall and this function moves the rover straight a little bit and turns around corner
# Parameters: N/A
def go_straight_and_corner_left():
    check_trigger()
    motor_forward()
    time.sleep(2)
    check_trigger()
    motor_stop()
    motor_spin_left()
    while sensor_left_corner >= sensor_min_reading:
        check_trigger()
    motor_stop()
    return

# Function: go_straight_until_detection_left
# Purpose: Make rover go straight until left sensor detects wall (and go straight for a small distance after detection)
# Parameters: N/A
def go_straight_until_detection_left():
    check_trigger()
    motor_forward()
    while sensor_left >= sensor_min_reading:
        check_trigger()
    time.sleep(0.5)
    motor_stop()
    if sensor_left_corner >= sensor_min_reading:
        motor_spin_left()
        while sensor_left_corner >= sensor_min_reading:
            check_trigger()
        motor_stop()
        if sensor_left >= sensor_min_reading:
            motor_spin_right()
            while sensor_left >= sensor_min_reading:
                check_trigger()
            time.sleep(0.5)
            motor_stop()
    check_trigger()
    return
# ********** WALL FOLLOWING ON RIGHT **********

# Function: wall_following_wall_on_right
# Purpose: Follow a wall when the right side of the rover is facing the wall
# Parameters: N/A
def wall_following_wall_on_right():
    check_trigger()
    while sensor_front > sensor_front_detection and bump_sensor_right < bump_sensor_trigger and bump_sensor_left < bump_sensor_trigger:
        while sensor_right < sensor_min_reading and sensor_right_corner < sensor_min_reading: # aka the right and right corner sensor reads that there is a wall
            check_trigger()
            motor_forward()
            c_equiv = sensor_right_corner * math.cos(math.radians(30.0))
            if sensor_right > c_equiv:
                if sensor_right_corner < (sensor_right * math.cos(math.radians(tolerance_angle)) - 12.0 * math.sin(math.radians(tolerance_angle))) / math.cos(math.radians(30.0 - tolerance_angle)):
                    turn_to_parallel_rightwall()
            elif sensor_right < c_equiv:
                if sensor_right_corner > (sensor_right * math.cos(math.radians(tolerance_angle)) + 12.0 * math.sin(math.radians(tolerance_angle))) / math.cos(math.radians(30.0 + tolerance_angle)):
                    turn_to_parallel_leftwall()
            check_trigger()
            motor_forward()
            dumby = sensor_right
            if sensor_min_r - dist_center_to_sensor < dumby < sensor_max_r - dist_center_to_sensor:
                check_trigger()
            elif dumby > sensor_max_r - dist_center_to_sensor:
                check_trigger()
                turn_to_parallel_rightwall()
                turn_right_towards_wall()
                go_straight_until_center_rightwall("toward")
                turn_to_parallel_rightwall()
                check_trigger()
            elif dumby < sensor_min_r - dist_center_to_sensor:
                check_trigger()
                turn_to_parallel_rightwall()
                turn_left_away_from_wall()
                go_straight_until_center_rightwall("away")
                turn_to_parallel_rightwall()
                check_trigger()
        motor_stop()
        check_trigger()
        if sensor_right < sensor_min_reading < sensor_right_corner:
            r_dist_maintain = sensor_right
            go_straight_only_right_sensor(r_dist_maintain)
        go_straight_and_corner_right()
        go_straight_until_detection_right()
        wall_following_wall_on_right()
    motor_stop()
    check_trigger()
    if sensor_front < sensor_front_detection or bump_sensor_right > bump_sensor_trigger or bump_sensor_left > bump_sensor_trigger:
        if bump_sensor_right > bump_sensor_trigger or bump_sensor_left > bump_sensor_trigger: # Maybe change up code later to take in corner sensor data
            motor_reverse()
            time.sleep(1) # one second delay
            motor_stop()
        main_code("left", 69)
    else:
        wall_following_wall_on_right()
    return

# Function: turn_to_parallel_rightwall
# Purpose: This function moves the rover parallel to the wall when the wall is on the right
# Parameters: N/A
def turn_to_parallel_rightwall():
    check_trigger()
    motor_stop()
    c_equiv = sensor_right_corner * math.cos(math.radians(30.0))
    if c_equiv > sensor_right:
        motor_spin_right()
        while sensor_right_corner * math.cos(math.radians(30.0)) > sensor_right:
            check_trigger()
    elif c_equiv < sensor_right:
        motor_spin_left()
        while sensor_right_corner * math.cos(math.radians(30.0)) < sensor_right:
            check_trigger()
    motor_stop()
    check_trigger()
    return

# Function: turn_right_towards_wall
# Purpose: When rover is parallel to wall in right wall following mode and is too far from the wall this will turn it towards the wall
# Parameters: N/A
# Notes: parameter angle can be changed at top of code
def turn_right_towards_wall():
    check_trigger()
    dist = sensor_right
    if dist < sensor_max_r - dist_center_to_sensor:
        return
    c_desired = (dist + dist_center_to_sensor - 12.0 * math.cos(math.radians(angle)) - 14.5 * math.sin(math.radians(angle))) / math.cos(math.radians(30.0 - angle))
    r_desired = (dist + dist_center_to_sensor - 12.0 * math.cos(math.radians(angle)) - 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
    motor_spin_right()
    while sensor_right < r_desired:
        check_trigger()
    if sensor_right > sensor_min_reading:
        motor_stop()
        motor_spin_right()
        while sensor_right_corner > c_desired:
            check_trigger()
    motor_stop()
    check_trigger()
    return

# Function: turn_left_away_from_wall
# Purpose: When rover is parallel to wall in right wall following mode and is too close to the wall this will turn it away from the wall
# Parameters: N/A
# Notes: parameter angle can be changed at top of code
def turn_left_away_from_wall():
    check_trigger()
    dist = sensor_right
    if dist > sensor_min_r - dist_center_to_sensor:
        return
    r_desired = (dist + dist_center_to_sensor - 12.0 * math.cos(math.radians(angle)) + 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
    motor_spin_left()
    while sensor_right < r_desired:
        check_trigger()
    motor_stop()
    return

# Function: go_straight_until_center_rightwall
# Purpose: After correction turn this function will move the rover into the center of the tolerances
# Parameters: direction, either "toward" or "away"
# Notes: sensor_min_r and sensor_max_r can be changed at top of code
def go_straight_until_center_rightwall(direction):
    check_trigger()
    if direction == "toward":
        r_desired = (0.5 * (sensor_min_r + sensor_max_r) - 12.0 * math.cos(math.radians(angle)) - 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
        motor_forward()
        while sensor_right > r_desired:
            check_trigger()
    elif direction == "away":
        r_desired = (0.5 * (sensor_min_r + sensor_max_r) - 12.0 * math.cos(math.radians(angle)) + 2.5 * math.sin(math.radians(angle))) / math.cos(math.radians(angle))
        motor_forward()
        while sensor_right < r_desired:
            check_trigger()
    motor_stop()
    return

# Function: go_straight_only_right_sensor
# Purpose: Rover is near end of a wall and right corner sensor no longer detects so this program maintains right sensor distance and goes straight until wall cleared
# Parameters: dist, the distance of the right sensor when function called
def go_straight_only_right_sensor(dist):
    check_trigger()
    if sensor_right < sensor_min_reading:
        motor_forward()
        while dist - 2 < sensor_right < dist + 2:
            check_trigger()
        if dist + 2 < sensor_right < sensor_min_reading:
            motor_spin_right()
            time.sleep(1)
            motor_stop()
            go_straight_only_right_sensor(dist)
        elif sensor_right < dist - 2:
            motor_spin_left()
            time.sleep(1)
            motor_stop()
            go_straight_only_left_sensor(dist)
    motor_stop()
    return

# Function: go_straight_and_corner_right
# Purpose: Right sensor cleared the wall and this function moves the rover straight a little bit and turns around corner
# Parameters: N/A
def go_straight_and_corner_right():
    check_trigger()
    motor_forward()
    time.sleep(2)
    check_trigger()
    motor_stop()
    motor_spin_right()
    while sensor_right_corner >= sensor_min_reading:
        check_trigger()
    motor_stop()
    return

# Function: go_straight_until_detection_right
# Purpose: Make rover go straight until right sensor detects wall (and go straight for a small distance after detection)
# Parameters: N/A
def go_straight_until_detection_right():
    check_trigger()
    motor_forward()
    while sensor_right >= sensor_min_reading:
        check_trigger()
    time.sleep(0.5)
    motor_stop()
    if sensor_right_corner >= sensor_min_reading:
        motor_spin_right()
        while sensor_right_corner >= sensor_min_reading:
            check_trigger()
        motor_stop()
        if sensor_right >= sensor_min_reading:
            motor_spin_left()
            while sensor_right >= sensor_min_reading:
                check_trigger()
            time.sleep(0.5)
            motor_stop()
    check_trigger()
    return




##function to set to manual mode
def set_mode_manual():
  rospy.wait_for_service('/mavros/set_mode')
  try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    mode='MANUAL'
    resp1 = change_mode(0,mode)
  except rospy.ServiceException as e:
    print ("Service call failed: %s" %e)
  pub.publish(msg)

def end_bug():
  rospy.wait_for_service('/mavros/set_mode')
  try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    mode='AUTO'
    resp1 = change_mode(0,mode)
  except rospy.ServiceException as e:
    print ("Service call failed: %s" %e)
  pub.publish(msg)
  collision_detected=0










###function to use motors
def motor_forward():
  pub=rospy.Publisher('rcout', bug2, queue_size=1) 
  msg=bug2()
  msg.forward=1
  pub.publish(msg)

def motor_reverse():
  pub=rospy.Publisher('rcout', bug2, queue_size=1) 
  msg=bug2()
  msg.reverse=1()
  pub.publish(msg)

def motor_spin_right():
  pub=rospy.Publisher('rcout', bug2, queue_size=1) 
  msg=bug2()
  msg.spinright=1
  pub.publish(msg)

def motor_spin_left():
  pub=rospy.Publisher('rcout', bug2, queue_size=1)
  msg=bug2()
  msg.spinleft=1
  pub.publish(msg)

def motor_turn_right():
  pub=rospy.Publisher('rcout', bug2, queue_size=1)
  msg=bug2()
  msg.turn_right=1
  pub.publish(msg)

def motor_turn_left():
  pub=rospy.Publisher('rcout', bug2, queue_size=1)
  msg=bug2()  
  msg.turnleft=1
  pub.publish(msg)

def motor_stop():
  pub=rospy.Publisher('rcout', bug2, queue_size=1)
  msg=bug2()  
  msg.forward=0
  msg.reverse=0
  msg.spinright=0
  msg.spinleft=0
  msg.turnright=0
  msg.turnleft=0
  pub.publish(msg)

if __name__=='__main__':
  bug2main()

