#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

def move_circle():
		""" Moves the circle in a circle with a constant twist velocity"""
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)
		
		velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		#subscriber = rospy.Subscriber('/turtle1/pose', Pose)
		rate = rospy.Rate(10)
		vel_msg = Twist()
		while not rospy.is_shutdown():
			# Linear velocity in the x-axis
			vel_msg.linear.x = 1
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			
			# Angular velocity in z-axis
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 1
			
			# Publishing our vel_msg
			velocity_publisher.publish(vel_msg)
			#print(subscriber.theta)
			# Publish at a desired rate
			rate.sleep()

		# if we press ctrl + c, the node will stop
		rospy.spin()
	
if __name__ == '__main__':
	try:
		move_circle()
	except rospy.ROSInterruptException:
		pass
