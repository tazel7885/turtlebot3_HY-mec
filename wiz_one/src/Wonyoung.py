#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import numpy as np
import actionlib
from actionlib_msgs.msg import GoalStatus
import math
import tf
from std_msgs.msg import String,Int8
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from tf.transformations import quaternion_from_euler
rospy.init_node('detect_tunnel')
# import listener

count = 0
goal_x = 0
goal_y = 0
init_x = 0
init_y = 0
initialPose = 0
stage = 0
pub_tunnel_return = 100

######################################################################### IZ_ONE ##############################################################################

class DetectTunnel():
    def cbOdom(self, odom_msg):

        # print("cbOdom")
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)
        self.odom_msg = odom_msg
        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

        # global x
        # global y
        global count
        global init_x
        global init_y
        global goal_x
        global goal_y

        # x = odom_msg.pose.pose.position.x
        # y = odom_msg.pose.pose.position.y
        
        
        if count < 1:
            init_x = -1.55
            init_y = -0.35
            goal_x = -0.05
            goal_y = -1.95
            # init_x = -1.765
            # init_y = -0.420
            # goal_x = -0.50
            # goal_y = -1.55
            rospy.loginfo("goal_x: %f", goal_x)
            count = 100


    def __init__(self):
        self.sub_arrival_status = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.cbGetNavigationResult, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        # self.pub_tunnel_return = rospy.Publisher('/chatter', String, queue_size=1)
        self.pub_goal_pose_stamped = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_init_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)
        self.pub = rospy.Publisher('/last',Int8,queue_size= 1)

        self.is_navigation_finished = False
        self.is_tunnel_finished = False

        self.last_current_theta = 0.0
        print("__init__")

       
        target_point = [goal_x, goal_y, 0] #[0.03,-1.77,0]
        yaw_euler_angles = [359]
        quat = list()
        #global dream_pose
        self.dream_pose = list()
        for yawangle in yaw_euler_angles:
            quat.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*3.141592/180, axes='sxyz'))))
        
        points = [target_point[i:i+3] for i in range(0, len(target_point), 3)]
        for point in points:
            self.dream_pose.append(Pose(Point(*point),quat[0]))

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")


    def cbGetNavigationResult(self, msg_nav_result):
        print("end!")
        if msg_nav_result.status.status == 3:
            rospy.loginfo("Reached")
            last = 7
            self.is_navigation_finished = True
            self.pub.publish(last)

    def cbTunnelOrder(self): # Publisher, Subscriber	
        # print("cbTunnelOrder")
        global pub_tunnel_return
        
        if pub_tunnel_return == 100:
            print("100")
            rospy.loginfo("Now lane_following")

            pub_tunnel_return = 200
                            
                                
        elif pub_tunnel_return == 200:
            print("200")
            rospy.loginfo("Now go_in_to_tunnel")

            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y

            # while True:
            #     error = self.fnStraight(0.55)

            #     if math.fabs(error) < 0.005:
            #         break

            # self.fnStop()

            rospy.loginfo("go_in_to_tunnel finished")

            pub_tunnel_return = 300


        elif pub_tunnel_return == 300:
            print("300")
            rospy.loginfo("Now navigation")
            initialPose = PoseWithCovarianceStamped()
            initialPose.header.frame_id = "map"
            initialPose.header.stamp = rospy.Time.now()
            # initialPose.pose.pose = self.odom_msg.pose.pose
            initialPose.pose.pose.position.x = init_x
            initialPose.pose.pose.position.y = init_y
            initialPose.pose.pose.position.z = 0.0

            initialPose.pose.pose.orientation.x = 0.0
            initialPose.pose.pose.orientation.y = 0.0
            initialPose.pose.pose.orientation.z = -0.70#-0.355
            initialPose.pose.pose.orientation.w = 0.714

            self.pub_init_pose.publish(initialPose)

            self.fnPubGoalPose()

            self.movebase_client()

            while True:
                if self.is_navigation_finished == True:
                    break
                else:
                    pass

            pub_tunnel_return = 400


        elif pub_tunnel_return == 400:
            print("400")
            rospy.loginfo("Now go_out_from_tunnel")

            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y

            while True:
                error = self.fnStraight(0.25)

                if math.fabs(error) < 0.005:
                    break

            self.fnStop()

            pub_tunnel_return = 500

        elif pub_tunnel_return == 500:
            print("Tunnel_finished")




    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta


    # def active_cb(self):
    #     rospy.loginfo("Goal pose is now being processed by the Action Server...")

    # def feedback_cb(self, feedback):
    #     rospy.loginfo("Feedback for goal pose received")

    # def done_cb(self, status, result):
    #     if status == 2:
    #         rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")

    #     if status == 3:
    #         rospy.loginfo("Goal pose reached") 

    #     if status == 4:
    #         rospy.loginfo("Goal pose was aborted by the Action Server")
    #         rospy.signal_shutdown("Goal pose aborted, shutting down!")
    #         return

    #     if status == 5:
    #         rospy.loginfo("Goal pose has been rejected by the Action Server")
    #         rospy.signal_shutdown("Goal pose rejected, shutting down!")
    #         return

    #     if status == 8:
    #         rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")

    def fnPubGoalPose(self):
        print("fnPubGoalPose")
        goalPoseStamped = PoseStamped()

        goalPoseStamped.header.frame_id = "map"
        goalPoseStamped.header.stamp = rospy.Time.now()

        goalPoseStamped.pose.position.x = goal_x
        goalPoseStamped.pose.position.y = goal_y
        goalPoseStamped.pose.position.z = 0.0

        goalPoseStamped.pose.orientation.x = 0.0
        goalPoseStamped.pose.orientation.y = 0.0
        goalPoseStamped.pose.orientation.z = 0.0
        goalPoseStamped.pose.orientation.w = 0.001

        self.pub_goal_pose_stamped.publish(goalPoseStamped)

    def movebase_client(self):
        print("movebase_client")
        print(self.dream_pose[0])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.dream_pose[0]
        rospy.loginfo("Sending goal pose to Action Server")
        # self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        # rospy.spin()

    def fnStraight(self, desired_dist):
        last = 7
        self.pub.publish(last)
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
        
        # rospy.loginfo("Tunnel_Straight")
        # rospy.loginfo("err_pos  desired_dist : %f  %f  %f", err_pos, desired_dist, self.lastError)

        Kp = 0.4
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def cbTunnelFinished(self, tunnel_finished_msg):
        self.is_tunnel_finished = True

def checking_stage(ddd):
    global stage
    stage = ddd.data
    print("stage=",stage)


def main():
    node = DetectTunnel()
    while True:
		if stage == 6:

			node.cbTunnelOrder()
			# node.cbOdom()
			# node.cbGetNavigationResult()

rospy.Subscriber('/stage', Int8, checking_stage,queue_size = 1)
main()
print(pub_tunnel_return)


