#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage

rospy.init_node("line2",anonymous=True)
pub = rospy.Publisher("/line2",Float32,queue_size=5)

######################################################################### IZ_ONE ##############################################################################

def line(data):
	lineM=[]
	M = 0

	i = 0
	#white
	np_arr = np.fromstring(data.data, np.uint8)
	line_cam = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	gray = cv2.cvtColor(line_cam, cv2.COLOR_BGR2GRAY)
	gau = cv2.GaussianBlur(gray, (15,15), 0)
	thr = cv2.adaptiveThreshold(gau, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)
	medi = cv2.medianBlur(thr, 3)
	edge = cv2.Canny(medi, 200, 250)
	mid = edge[110 : 230, 185: 240]
	M_lines = cv2.HoughLines(mid, 1, np.pi/180, 10)

	if M_lines is not None:
		M_lines=[l[0] for l in M_lines]
		for rho,theta in M_lines:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			degree=np.arctan2(y2-y1,x2-x1)*180/np.pi

			if  M==0:
				i+=1
				M+=1
				break
			else:
				continue


 	if i == 0:
 		pub.publish(200)
 	else:
 		pub.publish(100)
rospy.Subscriber("/camera1/usb_cam/image_raw/compressed",CompressedImage,line)

rospy.spin()