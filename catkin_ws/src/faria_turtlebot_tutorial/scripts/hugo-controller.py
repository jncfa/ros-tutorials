#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
##################################### Header ############################################
""" obstruction.py: Description of the node """
__author__ = ""
__credits__ = [""]
__version__ = "0.0.0"
__maintainer__ = ""
__email__ = ""
#########################################################################################
# import any libraries necessary to run script
import roslib
import rospy
import math
import time 
import sys
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *

class turtle():

	def __init__(self):
		self.x=0
		self.y=0
		self.yaw=0

		self.cmd_vel_topic = "cmd_vel"
		self.position_topic="pose"

		pose_subscriber = rospy.Subscriber(self.position_topic, Pose, self.poseCallback)

		controller=rospy.get_name()
		
		try:
			controller_spawn=controller+"/spawn_turtle_name"
			name=rospy.get_param(controller_spawn)
			self.spawner(3, 3 , 0, name)
		except Exception as e:
			print("ja existe")



		controller_vec=controller+"/linear_speed"
		controller_ang=controller+"/angular_speed"

		self.vec_ang=rospy.get_param(controller_ang)
		self.vec_linear=rospy.get_param(controller_vec)


		#self.move(vec_linear,20, True)
		self.go_to_goal(9,9)



	def spawner(self,x, y, theta, name):
		print("%s %s %s %s" %(x,y,theta,name))
		#rospy.wait_for_service('/spawn')
		try:
			placing = rospy.ServiceProxy('/spawn', Spawn)
			resp1 = placing(x, y, theta , name)
			return resp1.name
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	def poseCallback(self,pose_messsage):
		
		self.x=pose_messsage.x
		self.y=pose_messsage.y
		self.yaw=pose_messsage.theta

	def move(self,speed, distance, is_forward ):

		velocity_message = Twist()

		
		x0=self.x
		y0=self.y
		
		print ('x %f +++++ y %f  ' %(self.x,self.y))

		if(is_forward):
			velocity_message.linear.x=abs(speed)
		else:
			velocity_message.linear.x=-abs(speed)

		distance_moved = 0.0
		rate = rospy.Rate(10)

		velocity_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

		while True:
			rospy.loginfo("Turtlesim moves forwards")
			velocity_publisher.publish(velocity_message)

			rate.sleep()
			#rospy.Duration(1.0)

			distance_moved= distance_moved+ abs(0.5 * math.sqrt(((self.x-x0)**2) + ((self.y-y0)**2)))

			print (distance_moved)

			if not (distance_moved<distance):
				rospy.loginfo("reached")
				break

		#parar o robo quando a distancia é atingida 
		velocity_message.linear.x=0
		velocity_publisher.publish(velocity_message)

	def go_to_goal(self,x_goal, y_goal):

		velocity_message = Twist()


		velocity_publisher = rospy.Publisher(self.cmd_vel_topic, Twist ,queue_size=10)

		#print yaw
		#time.sleep(7)

		while(True):

			#K_linear=1
			distance= abs(math.sqrt(((x_goal-self.x)**2)+((y_goal-self.y)**2)))
			linear_speed=distance * self.vec_linear

			#K_angular = 10.0
			desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
			angular_speed = (desired_angle_goal-self.yaw)*self.vec_ang

			velocity_message.linear.x = linear_speed
			velocity_message.angular.z = angular_speed

			velocity_publisher.publish(velocity_message)

			print ("x= %s y= %s" %(self.x,self.y))
			#print desired_angle_goal
			#break

			if (distance < 0.05):
				print ("End Goal")
				break






def main():
	rospy.init_node('basic_control', anonymous=True)
	obst = turtle()
	rospy.spin()



if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		sys.exit(0)