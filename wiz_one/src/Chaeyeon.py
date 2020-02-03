#!/usr/bin/env python
import numpy as np 
import cv2
import rospy

######################################################################### IZ_ONE ##############################################################################

def parkingafter(parkingaftercam):
	#hsv
	ROI = parkingaftercam[:,:330]
	blur = cv2.blur(ROI, (9,9))
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	#mask direction
	lowerblue = np.array([100,130,0])
	upperblue = np.array([110,255,125])
	maskblue = cv2.inRange(hsv, lowerblue, upperblue)
	#contour
	_, contourblue, _ = cv2.findContours(maskblue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


	if len(contourblue) > 0:
		for i in range(len(contourblue)):
			areablue = cv2.contourArea(contourblue[i])
			print(areablue)
			if areablue >= 2800:
				return 7

			else:
				return 100
	else:
		return 100