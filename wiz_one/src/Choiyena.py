#!/usr/bin/env python
import cv2
import numpy as np 
import rospy

from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)

######################################################################### IZ_ONE ##############################################################################


def parking_stanby(parkingcam):
	center = []
	c = 0
	Cdegree = 0

	gray = cv2.cvtColor(parkingcam, cv2.COLOR_BGR2GRAY)
	gau = cv2.GaussianBlur(gray, (15,15), 0)
	thr = cv2.adaptiveThreshold(gau, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)
	medi = cv2.medianBlur(thr, 3)
	edge = cv2.Canny(medi, 200, 250)
	center_edge = edge[120 : 230, 210: 220]
	C_lines = cv2.HoughLines(center_edge, 1, np.pi/180, 5)

	if C_lines is not None:
		C_lines=[l[0] for l in C_lines]
		for rho,theta in C_lines:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			degree=np.arctan2(y2-y1,x2-x1)*180/np.pi
			if  c==0:
				Cdegree=degree
				print(degree)
				c+=1

				break
			else:
				continue
	if c == 0:
		return 100
	else:
		return 10



def obstacle_moving():
	rospy.sleep(0.2)
	turtlego(0, 1.6)
	rospy.sleep(0.5)
	turtlego(0, 1.6)
	rospy.sleep(0.4)

	turtlego(2.0,0)
	rospy.sleep(0.5)
	turtlego(2.0,0)
	rospy.sleep(0.5)
	turtlego(2.0,0)
	rospy.sleep(0.5)

	turtlego(0, -1.6)
	rospy.sleep(0.5)
	turtlego(0, -1.6)
	rospy.sleep(0.25)

	return 3001
def obstacle_moving2():
	rospy.sleep(0.2)	
	turtlego(0, -1.6)
	rospy.sleep(0.5)
	turtlego(0, -1.6)
	rospy.sleep(0.5)

	turtlego(2.0,0)
	rospy.sleep(0.5)
	turtlego(2.0,0)
	rospy.sleep(0.5)
	turtlego(2.0,0)
	rospy.sleep(0.4)

	turtlego(0, 1.6)
	rospy.sleep(0.5)
	turtlego(0, 1.6)
	rospy.sleep(0.5)

	return 3
def turtlego(linear, angle):

	twist = Twist()
	twist.linear.x = linear
	twist.angular.z = angle
	pub.publish(twist)

