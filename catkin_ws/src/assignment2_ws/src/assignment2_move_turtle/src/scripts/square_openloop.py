#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def move_square():
    """Moves in a square configuration with a constant velocity of 0.2 units/s"""
    # Creates a node called "turtlebot_square" and makes sure its unique
    rospy.init_node('turtlebot_square', anonymous=True)
    
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    t0 = rospy.Time.now().to_sec()
    t1 = t0
    
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    while not rospy.is_shutdown():
        
        while t1 - t0 <= 10:
            vel_msg.linear.x = 0.2
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            rate.sleep()
            
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)
        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()
        while t1 - t0 <= 7.853981634:
            vel_msg.angular.z = 0.2
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            rate.sleep()
            
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()
        
    # if we press ctrl+c, the node will shut down
    rospy.spin()
    
if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass    
