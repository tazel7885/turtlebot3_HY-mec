#!/usr/bin/env python
import numpy as np 
import cv2
import rospy

lowerred = np.array([170,160,0])
upperred = np.array([180,255,255])

lowerYellow = np.array([15,85,0])
upperYellow = np.array([30,255,255])
######################################################################### IZ_ONE ##############################################################################

def chadan(chadancam):


	blur = cv2.blur(chadancam, (7,7))
	chadan = blur[:,:200]
	hsv1 = cv2.cvtColor(chadan,cv2.COLOR_BGR2HSV)
	hsv2 = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
	maskred = cv2.inRange(hsv1, lowerred, upperred)
	#maskyellow = cv2.inRange(hsv2,lowerYellow,upperYellow)
	_, contourred, _ = cv2.findContours(maskred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#_, contouryellow, _ = cv2.findContours(maskyellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(contourred) > 0:
		for i in range(len(contourred)):
			areared = cv2.contourArea(contourred[i])
			print("red",areared)
			if areared >= 300 :
				return 8
			# elif areared >= 100 and areared < 400:
			# 	if len(contouryellow) > 0:
			# 		for k in range(len(contouryellow)):
			# 			areayellow = cv2.contourArea(contouryellow[k])
			# 			print("yell",areayellow)
			# 			if areayellow > 400:
			# 				return 9
			# 			else:
			# 				return 100
			# 	else:
			# 		return 100	
			else:
				return 100

	else:
		return 100

