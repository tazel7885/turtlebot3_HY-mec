#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)

######################################################################### IZ_ONE ##############################################################################

def parking_motion_left():

	rospy.sleep(0.2)
	turtlego(0, -1.7)
	rospy.sleep(0.5)
	turtlego(0, -1.7)
	rospy.sleep(0.5)
	turtlego(-2.0,0)
	rospy.sleep(0.5)
	turtlego(-2.0,0)
	rospy.sleep(0.4)
	turtlego(-2.0,0)
	rospy.sleep(0.2)
	turtlego(3.0,-1.5)
	rospy.sleep(0.5)
	turtlego(3.0,-1.5)
	rospy.sleep(0.5)
	turtlego(3.0,-1.5)
	rospy.sleep(0.5)
	turtlego(3.0,-1.5)
	rospy.sleep(0.5)
	turtlego(3.0,-1.5)
	rospy.sleep(0.5)
	return 5000

def parking_motion_right():
	rospy.sleep(0.2)
	turtlego(0, 1.6)
	rospy.sleep(0.5)
	turtlego(0, 1.6)
	rospy.sleep(0.5)
	turtlego(-2.0,0)
	rospy.sleep(0.5)
	turtlego(-2.0,0)
	rospy.sleep(0.4)
	turtlego(-2.0,0)
	rospy.sleep(0.2)
	turtlego(3.0,1.5)
	rospy.sleep(0.5)
	turtlego(3.0,1.5)
	rospy.sleep(0.5)
	turtlego(3.0,1.5)
	rospy.sleep(0.5)
	turtlego(3.0,1.5)
	rospy.sleep(0.5)
	turtlego(3.0,1.5)
	rospy.sleep(0.3)
	return 5000
	
def turtlego(linear, angle):

	twist = Twist()
	twist.linear.x = linear
	twist.angular.z = angle
	pub.publish(twist)