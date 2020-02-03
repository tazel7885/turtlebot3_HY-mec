#!/usr/bin/env python
import cv2
import numpy as np
import rospy

lowerred = np.array([170,160,0])
upperred = np.array([180,255,255])

lowerYellow = np.array([15,110,0])
upperYellow = np.array([30,255,255])

######################################################################### IZ_ONE ##############################################################################

def construct (construct_cam):
	blur = cv2.blur(construct_cam, (7,7))
	hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
	roi = hsv[:,280:395]
	maskred = cv2.inRange(roi, lowerred, upperred)
	maskyellow = cv2.inRange(roi,lowerYellow,upperYellow)
	_, contourred, _ = cv2.findContours(maskred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	_, contouryellow, _ = cv2.findContours(maskyellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


	if len(contourred) > 0:
		for cnt in contourred:
			for i in range(len(contourred)):
				areared = cv2.contourArea(contourred[i])

				if areared > 10:

					for k in range(len(contouryellow)):
						areayellow = cv2.contourArea(contouryellow[k])

						if areayellow > 100:
							return 3
						else:
							return 100

				else:
					return 100
	else:
		return 100

