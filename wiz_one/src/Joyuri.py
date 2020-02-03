#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage

rospy.init_node("line",anonymous=True)
pub = rospy.Publisher("/line",Float32,queue_size=5)

######################################################################### IZ_ONE ##############################################################################

def line(data):
	lineL=[]
	lineR=[]
	L = 0
	R = 0
	Rdegree=0
	Ldegree=0
	i = 0
	#white
	np_arr = np.fromstring(data.data, np.uint8)
	line_cam = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	
	gray = cv2.cvtColor(line_cam, cv2.COLOR_BGR2GRAY)
	gau = cv2.GaussianBlur(gray, (15,15), 0)
	thr = cv2.adaptiveThreshold(gau, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)
	medi = cv2.medianBlur(thr, 3)
	edge = cv2.Canny(medi, 230, 250)
	
	right_edge = edge[220 : 240, 215 :430]
	left_edge = edge[220 : 240, : 215]

	L_lines = cv2.HoughLines(left_edge, 1, np.pi/180, 10)
	R_lines = cv2.HoughLines(right_edge, 1, np.pi/180, 10)

	if R_lines is not None:
		R_lines=[l[0] for l in R_lines]
		for rho,theta in R_lines:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			degree=np.arctan2(y2-y1,x2-x1)*180/np.pi

			if  R==0:
				i+=1
				Rdegree=degree

				R+=1

				break
			else:
				continue

	if L_lines is not None:
		L_lines=[l[0] for l in L_lines]
		for rho,theta in L_lines:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			degree=np.arctan2(y2-y1,x2-x1)*180/np.pi

			if  L == 0:
				i = i + 1

				Ldegree=degree
				L+=1

				break
			else:
				continue

	#cv2.imshow("frucjcksajdfas", frame)
	if i == 2:
		pub.publish((Ldegree + Rdegree) * 0.008)

	elif i == 1:
		if R > 0:
			if Rdegree > 50:
				pub.publish((Rdegree - 50) * 0.045)
			elif Rdegree > 0 and Rdegree <= 50:
				pub.publish(Rdegree * 0.035)
			else:
				pub.publish(-Rdegree * 0.035)
		else:
			if Ldegree < -50:
				pub.publish((Ldegree + 50) * 0.045)
			elif Ldegree < 0 and Ldegree >= -50:
				pub.publish(Ldegree * 0.035)
			else:
				pub.publish(-Ldegree * 0.035)


	else:
		pub.publish(100)
 
rospy.Subscriber("/camera1/usb_cam/image_raw/compressed",CompressedImage,line)

rospy.spin()
