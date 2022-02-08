#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_square_closed' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_square_closed', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=10):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = [Pose(), Pose(), Pose(), Pose()]
        param_mat = ['~x1','~x2','~x3','~x4','~y1','~y2','~y3','~y4']
        ind = 0
        i = 0
        for goal in goal_pose:
            goal.x = rospy.get_param(param_mat[ind])
            goal.y = rospy.get_param(param_mat[ind+4])
            ind = ind + 1

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = rospy.get_param('~tol')

        vel_msg = Twist()
        while not rospy.is_shutdown():

            while self.euclidean_distance(goal_pose[i%4]) > distance_tolerance:

                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control
    
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(goal_pose[i%4])
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
    
                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose[i%4])
    
                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
    
                # Publish at the desired rate.
                self.rate.sleep()

            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            i = (i+1)%4
            # Publish at the desired rate.
            self.rate.sleep()

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
