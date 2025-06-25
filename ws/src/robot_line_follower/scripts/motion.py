#!/usr/bin/env python

# ROS packages (python)
import rospy

from geometry_msgs.msg import Twist

class MotionPlanner:
    def __init__(self):
        self.velocity = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def move(self, error):

        print(error)
        error = 0.01 * error #scale to m
        self.velocity.angular.z = error * -0.1

        vel_lin = 0.8
        if(error != 0):
           vel_lin =  0.1 * (1/abs(error))
        if(vel_lin > 0.8 ): vel_lin = 0.8
        self.velocity.linear.x = vel_lin

        self.publisher.publish(self.velocity)