#!/usr/bin/env python
import cv2
import numpy as np
import rospy

lowerred = np.array([170,100,0])
upperred = np.array([180,255,255])

lowerYellow = np.array([10,100,0])
upperYellow = np.array([30,255,255])

######################################################################### IZ_ONE ##############################################################################

def tunnel_sign (tunnel_cam):
	blur = cv2.blur(tunnel_cam, (7,7))
	hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
	maskred = cv2.inRange(hsv, lowerred, upperred)
	maskyellow = cv2.inRange(hsv,lowerYellow,upperYellow)
	_, contourred, _ = cv2.findContours(maskred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	_, contouryellow, _ = cv2.findContours(maskyellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	# cv2.imshow("red",maskred)	
	# cv2.waitKey(1)	

	
	# cv2.imshow("yellow",maskyellow)
	# cv2.waitKey(1)
	# cv2.imshow("cam",tunnel_cam)
	# cv2.waitKey(1)
	if len(contourred) > 0:
		for cnt in contourred:
			for i in range(len(contourred)):
				areared = cv2.contourArea(contourred[i])
				print("red",areared)
				if areared > 50:
					if len(contouryellow) > 0:
						for k in range(len(contouryellow)):
							areayellow = cv2.contourArea(contouryellow[k])
							print("yell",areayellow)
							if areayellow > 200:
								return 9
							else:
								return 100
					else:
						return 100	
				else:
					return 100
	else:
		return 100