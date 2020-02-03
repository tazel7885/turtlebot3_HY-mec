#!/usr/bin/env python
import cv2
import numpy as np
import rospy

######################################################################### IZ_ONE ##############################################################################


def parking(parkingcam):
	#image of parking sign
	img = cv2.imread('/home/hy-mec/Desktop/parking.jpg', cv2.IMREAD_GRAYSCALE)
	#blur,gray
	gray = cv2.cvtColor(parkingcam, cv2.COLOR_BGR2GRAY)
	#hsv
	blur = cv2.blur(parkingcam,(9,9))
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	#mask parking
	lowerblue = np.array([100,130,0])
	upperblue = np.array([110,255,125])
	maskblue = cv2.inRange(hsv, lowerblue, upperblue)
	#contour
	_, contourblue, _ = cv2.findContours(maskblue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#find direction


	if len(contourblue) > 0:
		#Feature matching parking sign
		for i in range(len(contourblue)):
			areablue = cv2.contourArea(contourblue[i])
			if areablue >= 3900:
				# surf = cv2.xfeatures2d.SURF_create()
				# kp1, des1 = surf.detectAndCompute(img, None)
				# kp2, des2 = surf.detectAndCompute(gray, None)

				# FLANN_INDEX_KDTREE = 0
				# index = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
				# search = dict(checks = 50)
				# flann = cv2.FlannBasedMatcher(index, search)
				# matches = flann.knnMatch(des1, des2, k=2)

				# good = []
				# for m,n in matches:
				# 	if m.distance < 0.5*n.distance:
				# 		good.append(m)
				# # res = cv2.drawMatches(img, kp1, gray, kp2, good, res, flags = 2)
				# #print(len(good))
				# if len(good) >= 20:
				# 	return 4
				# else:
				# 	print(len(good))
					return 4
			else:
				return 100
	else:
		return 100