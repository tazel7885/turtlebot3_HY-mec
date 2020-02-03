#!/usr/bin/env python
import numpy as np 
import cv2
import rospy

######################################################################### IZ_ONE ##############################################################################

def direction(directioncam):
	#hsv
	blur = cv2.blur(directioncam,(7,7))
	median = cv2.medianBlur(blur,3)
	hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
	#mask direction
	lowerblue = np.array([100,125,0])
	upperblue = np.array([110,255,255])
	maskblue = cv2.inRange(hsv, lowerblue, upperblue)
	#contour
	_, contourblue, _ = cv2.findContours(maskblue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#find direction

	if len(contourblue) > 0:
		for i in range(len(contourblue)):
			areablue = cv2.contourArea(contourblue[i])
			#print(areablue)
			if areablue >= 1800 and areablue < 4000:
				return 1000
			elif areablue >= 4000:
				x, y, w, h = cv2.boundingRect(contourblue[i])
				# print(x,w,h)
				if w >= 70 and h >= 65:
					if x >= 145 :
						#print("right")
						return 2

					elif x < 145 and x >= 0 :
						#print("left")
						return 1
					else:
						return 100
				else:
					return 100
			elif areablue < 200 and areablue > 0:
			 	i = i + 1
			else:
				return 100
	else:
		return 100

