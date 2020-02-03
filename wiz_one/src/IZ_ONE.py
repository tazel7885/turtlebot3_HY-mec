#!/usr/bin/env python
import cv2
import numpy as np 
import rospy
import math


#################################################################### IZ_ONE###############################################################################

import Sakura  
import Chaewon
import Minju
import Nako
import Hyewon
import Choiyena
import Chaeyeon
import Eunbi
#import Hitomi
#import Joyuri
#import Wonyoung
#import Anyujin

###############################################################################################################################################################

from sensor_msgs.msg import CompressedImage, LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

rospy.init_node("main", anonymous=True)

pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)
pub_stage = rospy.Publisher('/stage',Int8,queue_size=1)
pub_imu_yaw = rospy.Publisher('IMU_Yaw',Float32,queue_size=10)
################################################################################################################################################################

left_distance=0
mid_distance=0
right_distance=0

stage = 0
count = 0

angular = 0
line = 0
line2 = 0

sinho = 0
fuck = 0
hell = 0
you = 0
last_count = 0
yaw_d = 0

Left = []
Right = []
Center = []
res_Left = []
res_Right = []
res_Center = []

l = 0
r = 0
c = 0
i_z = 0
g_z = 0
imu_z = 0.0
how = 0
endo = 0

check_l=0
check_r=0
check_m = 0
last_move = 1

#################################################################################################################################################################

def checking_sinho(data1):
	global sinho
	sinho= data1.data
	

def checking_line(data3):
	global angular
	angular= data3.data

def imu_sensor(data11):
	global imu_z
	imu_z = data11.orientation.z
	# print(imu_z)
def euler(data12):
	global yaw_d
	list_orientation = [data12.orientation.x, data12.orientation.y, data12.orientation.z, data12.orientation.w]
	roll, pitch, yaw = euler_from_quaternion(list_orientation)
	yaw_d = yaw *(180/math.pi)
	# print(yaw*(180/math.pi))
##########################################################################################################################################################################

def camera (data):
	global stage
	global count 
	global angular
	global line
	global fuck
	global last_count
	global r
	global last_move
	global you
	global i_z
	global g_z
	global imu_z
	global yaw_d

	#camera node setting
	np_arr = np.fromstring(data.data, np.uint8)
	frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	
	#algorithm
	if count == 0: #start
		if sinho == 0: #stop
			turtlego(0.0,0.0)
		else: #normal move , start (if find green sign)
			count = 1
			i_z = yaw_d
			#print(i_z)
			if i_z < 0:
				g_z = i_z + 180
			else:
				g_z = i_z - 180


	elif count == 1:#find bluesign
		stage = Sakura.direction(frame)
		if stage == 1000 and line2 == 100: #slowly
			print("slow")
			count = 1200
		else: #normal move( if do not find direction)
			if angular == 100:
				turtlego(2.5,0)
			else:
				turtlego(2.5,angular)
	elif count == 1100:
		stage = Sakura.direction(frame)
		if you < 1000:
			if stage == 1:
				count = 1000
			elif stage == 2:
				count = 1000
			else:
				turtlego(0,0)
				you = you + 1
		else:
			count = 1000
	elif count == 1200:
		if yaw_d < g_z + 10 and yaw_d > g_z - 10:
			count = 1100
		else:
			turtlego(0, 0.1)

	elif count == 1000: #search direction sign
		# stage = Sakura.direction(frame)
		print(stage)
		if stage == 1: #find left direction 
			print("left")
			rospy.sleep(0.3)
			turtlego(2.0,10)
			rospy.sleep(0.5)
			turtlego(2.0,10)
			rospy.sleep(0.3)
			turtlego(2.0, 10)
			rospy.sleep(0.2)
			line = 1
			count = 2000
		elif stage == 2:#find right direction
			print("right")
			rospy.sleep(0.3)
			turtlego(2.0,-10)
			rospy.sleep(0.5)
			turtlego(2.0,-10)
			rospy.sleep(0.3)
			turtlego(2.0,-10)
			rospy.sleep(0.5)
			line = 2
			count = 2000
		else:
			if angular == 100:
				turtlego(0.5,0.5)
			else:
				turtlego(2.5, angular)
				print('aj')

	elif count == 2000: #search construct sign
		stage = Minju.construct(frame)
		if stage == 3: #find construct sign
			print("construct")
			count = 2 #return common move
		else:
			if angular == 100: #Uturn and escape
				if line == 1:
					turtlego(2.5, 1.3)
				elif line == 2:
					turtlego(2.5, -1.3)
			else:
				turtlego(2.5,angular)


	elif count == 2:#search obstacle
		stage = obstacle()
		if stage == 3: #detect obstacle
			print("obstacle")
			count = 3000 
		elif stage == 100:
			if angular == 100:
				turtlego(2.5, -0.5)
			else:
				turtlego(2.5, angular)

	elif count == 3000: #obstacle avoid moving
		print("start")
		count = Choiyena.obstacle_moving()

	elif count == 3001:
		stage = obstacle()
		if stage == 3: #detect obstacle
			print("obstacle")
			count = 3002 
		elif stage == 100:
			if angular == 100:
				turtlego(2.5, 0.5)
			else:
				turtlego(2.5, angular)

	elif count == 3002:
		count = Choiyena.obstacle_moving2()

	elif count == 3:#search parking sign
		stage = Chaewon.parking(frame)
		if stage == 4: # find parking sign
			print("parking sign")
			count = 4200
		elif stage == 100:
			if angular == 100:
				turtlego(1.0, -0.5) 	
			else:
				turtlego(1.0, angular)

	elif count == 4200: #enter before parking area
		print('angular',angular)
		if angular == 100:
			rospy.sleep(0.2)			
			turtlego(2.5, 0)
			rospy.sleep(0.2)
			turtlego(2.5, 1.7)
			rospy.sleep(0.5)
			turtlego(2.5, 1.7)
			rospy.sleep(0.5)
			turtlego(2.5, 1.7)
			rospy.sleep(0.5)
			count= 4400 	
		else:
			turtlego(2.5, angular)

	elif count == 4400: #enter before parking area
		if fuck < 200:
			if angular == 100:
				turtlego(2.5,0)
				fuck = fuck + 1
			else:
				turtlego(2.5, angular)
				fuck = fuck + 1
		else:
			count = 4500

	# elif count == 4300: #parking stand by
	# 	stage = line2
	# 	print("start")
	# 	if stage == 100:
	# 		turtlego(0,0)
	# 		rospy.sleep(0.2)
	# 		count = 4500
	# 	else:
	# 		if angular == 100:
	# 			turtlego(2.5, 0)
	# 		else:
	# 			turtlego(2.5,angular)

	elif count == 4500: #parking motion
		stage = parking()
		if stage == 5 and line2 == 100: #parking motion start(left)
			print("leftparking")
			count = Hyewon.parking_motion_left()
		elif stage == 6 and line2 == 100:#parking motion start(right)
			print("rightparking")
			count = Hyewon.parking_motion_right()
		else:
			if angular == 100:
				turtlego(2.5, 0)
			else:
				turtlego(2.5,angular)

	elif count == 5000: #search blue left sign
		# stage = Chaeyeon.parkingafter(frame)
		if fuck < 200:
			fuck = fuck + 1
			if angular == 100:
				turtlego(1.0, 0)
			else:
				turtlego(2.5, angular)
		else:
			if line2 == 100: #find blue left sign
				print("find bluesign")
				rospy.sleep(0.3)
				turtlego(2.0,10)
				rospy.sleep(0.5)
				turtlego(2.0,10)
				rospy.sleep(0.3)
				turtlego(2.0,10)
				rospy.sleep(0.5)
				count = 4
			else:
				if angular == 100:
					turtlego(1.0, 0)
				else:
					turtlego(2.5, angular)

	elif count == 4: #search chadan, tunnel sign

		#count = Eunbi.tunnel_sign(frame)
		stage = Nako.chadan(frame) #chadan #tunnel_sign

		# print("stage:",stage)
		# print("count:",count)
		if stage == 8: #find chadan
			print("chadan")
			turtlego(0.0,0.0)
			rospy.sleep(5.0)
			count = 5
		# elif stage == 9:
		# 	count = 5
		else:
			if angular == 100:
				turtlego(2.5, 0)
			else:
				turtlego(2.5, angular)

	# elif count == 5555:
	# 	print("search tunnel")
	# 	stage = Eunbi.tunnel_sign(frame)
	# 	if stage == 9:
	# 		print("find tunnel")
	# 		count = 5
	# 	else:
	# 		if angular == 100:
	# 			turtlego(2.5, 0)
	# 		else:
	# 			turtlego(2.5, angular)


	elif count == 5: #enter tunnel
		print("in")
		if r >= 900 and angular == 100: #end line
		
			rospy.sleep(0.2)
			turtlego(1.0,0)
			rospy.sleep(0.3)
			# turtlego(2.5,0)
			# rospy.sleep(0.3)
			# turtlego(0, 1.6)
			# rospy.sleep(0.4)
			# turtlego(2.0,0)
			# rospy.sleep(0.2)
			# turtlego(0.0,0)
			# rospy.sleep(1000)
			count = 6
			
		else:
			if angular == 100:
				turtlego(2.5, 0)
			else:
				turtlego(2.5, angular)

	elif count == 6: #after tunnel
		pub_stage.publish(count)
		print(count)
		if last_count == 7:
			print("last_count")
			if last_move == 1:
				turtlego(0.1,0)
				if mid_distance < 0.25:
					rospy.sleep(0.2)
					turtlego(0, -1.7)
					rospy.sleep(0.5)
					turtlego(0, -1.7)
					rospy.sleep(0.5)
					last_move = 2

			elif last_move == 2:
				turtlego(0.1,0)
				if mid_distance < 0.25:
					rospy.sleep(0.2)
					turtlego(0, 1.6)
					rospy.sleep(0.5)
					turtlego(0, 1.6)
					rospy.sleep(0.5)
					last_move = 3
			else:
				if angular == 100:
					turtlego(2.5, 0)
				else:
					turtlego(2.5, angular)
###########################################################################################################################################################

def turtlego(linear, angular):

	twist = Twist()
	twist.linear.x = linear
	twist.angular.z = angular
	pub.publish(twist)

def DMS_L(left):
	global left_distance
	global l
	global Left
	global res_Left

	left_distance = left.data

	if len(Left) >= 5:
		res_Left.sort()
		l = (res_Left[1] + res_Left[2] + res_Left[3])/3
		del Left[0]
		res_Left[0:3] = Left[0:3]
		del res_Left[4]
	else:
		Left.append(left_distance)
		res_Left.append(left_distance)
	# print("dist_l:",l)

def DMS_C(center):
	global center_distance
	global c
	global Center
	global res_Center

	center_distance = center.data

	if len(Center) >= 5:
		res_Center.sort()
		c = (res_Center[1] + res_Center[2] + res_Center[3])/3
		del Center[0]
		res_Center[0:3] = Center[0:3]
		del res_Center[4]
	else:
		Center.append(center_distance)
		res_Center.append(center_distance)
	# print("dist_c:",c)

def DMS_R(right):
	global right_distance
	global r 
	global Right
	global res_Right

	right_distance = right.data
	
	if len(Right) >= 5:
		res_Right.sort()
		r = (res_Right[1] + res_Right[2] + res_Right[3])/3
		del Right[0]
		res_Right[0:3] = Right[0:3]
		del res_Right[4]
	else:
		Right.append(right_distance)
		res_Right.append(right_distance)
	# print("dist_r:",r)

def lidar(msg):
	global mid_distance
	mid_distance = msg.ranges[0]


def obstacle():
	global mid_distance
	global hell
	# print("mid_distance",mid_distance)

	if mid_distance <= 0.3 and mid_distance > 0 and hell ==0:
		hell = 1
		return 3
	elif mid_distance <= 0.3 and mid_distance > 0 and hell == 1:
		return 3
	else:
		return 100

def parking():
	global check_r
	global r
	global check_l
	global l 

	if l >= 600:
		# check_l += 1
		# if check_l > 10:
		return 6
		# else:
		# 	return 100

	elif r >= 600:
		# check_r += 1
		# if check_r > 10:
		return 5
		# else:
		# 	return 100
	else:
		# check_l = 0
		# check_r = 0
		return 100		

def checking_line_2(data10):
	global line2
	line2 = data10.data

def last(last):
	global last_count
	last_count = last.data



##################################################################################################################################################################

rospy.Subscriber("/camera2/usb_cam/image_raw/compressed",CompressedImage,camera)
rospy.Subscriber("/start",Int8, checking_sinho)
rospy.Subscriber("/line",Float32,checking_line)
rospy.Subscriber('/DMS_L',Float32,DMS_L)
rospy.Subscriber('/DMS_R',Float32,DMS_R)
rospy.Subscriber('/scan', LaserScan, lidar)
rospy.Subscriber("/line2",Float32,checking_line_2)
rospy.Subscriber("/last",Int8,last)
rospy.Subscriber("imu", Imu, euler)
rospy.spin()

#################################################################################################################################################################
