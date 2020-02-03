#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8

pub = rospy.Publisher("/start",Int8,queue_size=5)
rospy.init_node("start", anonymous=True)

######################################################################### IZ_ONE ##############################################################################

def traffic(data):


	np_arr = np.fromstring(data.data, np.uint8)
	traffic_cam = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	hsv = cv2.cvtColor(traffic_cam, cv2.COLOR_BGR2HSV)
	lowerGreen = np.array([50, 70, 0])
	upperGreen = np.array([80, 255, 255])

	GmaskG = cv2.inRange(hsv, lowerGreen, upperGreen)
	kernel = np.ones((11,11), np.uint8)

	GmaskG= cv2.morphologyEx(GmaskG, cv2.MORPH_OPEN, kernel)
	GmaskG = cv2.morphologyEx(GmaskG, cv2.MORPH_CLOSE, kernel)
	ret2, thrG = cv2.threshold(GmaskG, 127, 255, 0)

	_, contoursG, _ = cv2.findContours(thrG, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	if len(contoursG) > 0 :
		for i in range(len(contoursG)):
		# Get area value
			area = cv2.contourArea(contoursG[i])

			if area > 100:  # minimum yellow area
				(x,y), r = cv2.minEnclosingCircle(contoursG[i])
				center = (int(x), int(y))
				r = int(r)
				cv2.circle(traffic_cam, center, r, (0,0,255),3 )
				GREEN = cv2.circle(traffic_cam, center, r, (0,0,255),3 )
				if GREEN is not None:

					GREEN = None
					pub.publish(100)
	else:
		pub.publish(0)

rospy.Subscriber("/camera1/usb_cam/image_raw/compressed",CompressedImage,traffic)
rospy.spin()