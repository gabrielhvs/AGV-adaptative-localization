#!/usr/bin/env python3
import rospy
import yaml
import math
import os
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from tf.transformations import euler_from_quaternion

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, error, dt = 0.01):
            
            if(dt == 0): dt = 0.01

            # Proportional term
            P_out = self.Kp * error

            # Integral term
            self.integral += error * dt
            I_out = self.Ki * self.integral

            # Derivative term
            derivative = (error - self.previous_error) / dt
            D_out = self.Kd * derivative

            # Compute total output
            output = P_out + I_out + D_out

            # Update previous error
            self.previous_error = error

            return output

class VirtualLineFollower:
    def __init__(self):
        rospy.init_node('virtual_line_follower')

        online = rospy.get_param('~online', '')

        if(online):
            rospy.Subscriber("/waypoints/poses", PoseArray, self.pose_array_callback, queue_size=1)
        else:
            # Load line points from YAML
            self.load_line_points()

        # Publisher to cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to AMCL pose
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        # Robot current pose
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # Waypoint index
        self.current_index = 0
        self.line_points = []

        self.distance_threshold = 0.05
        self.angle_threshold = 0.05
        self.linear_speed = 0.4
        self.angular_speed_gain = 1.5
        self.max_angular_speed = 0.4

        rospy.loginfo("Virtual Line Follower Node Initialized")
        rospy.spin()

    def load_line_points(self):
        path = rospy.get_param('~line_path', '')
        if path == '':
            rospy.logerr("Parameter line_path not set")
            rospy.signal_shutdown("Parameter line_path not set")
            return

        try:
            with open(path, 'r') as file:
                data = yaml.safe_load(file)
                self.line_points = data['line_points']
                rospy.loginfo(f"Loaded {len(self.line_points)} waypoints")
        except Exception as e:
            rospy.logerr(f"Failed to load line file: {e}")
            rospy.signal_shutdown("Line file not found or incorrect format")

    def pose_array_callback(self, msg):
        points = []
        # Example: Print positions
        for i, pose in enumerate(msg.poses):
            _,_,yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            points.append([pose.position.x, pose.position.y, yaw])

        if (len(self.line_points) == 0):
            rospy.loginfo("Control loop")
            self.line_points = points
            self.control_loop()

    def pose_callback(self, msg):
        # Get current robot position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (heading)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):

        angle_control = PIDController(1.0, 0, 0)
        linear_control = PIDController(1.0, 0, 0)
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
                rospy.logwarn("Waiting for robot pose...")
                rate.sleep()
                continue

            if self.current_index >= len(self.line_points):
                rospy.loginfo("Line completed. Stopping robot.")
                self.send_velocity(0, 0)
                break

            goal_x, goal_y, goal_yaw = self.line_points[self.current_index]

            # Compute distance and angle to goal
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)

            final_yaw_error = self.normalize_angle(goal_yaw - self.robot_yaw)

            if distance > 0.05:
                linear = linear_control.compute(distance)
                angular = angle_control.compute(angle_error)
            elif final_yaw_error < 0.05:
                rospy.loginfo(f"Reached waypoint {self.current_index + 1}")
                self.current_index += 1
                continue
            else:
                linear = 0.0
                angular = 1 * final_yaw_error

            angular = max(min(angular, self.max_angular_speed), -self.max_angular_speed)

            rospy.loginfo(f"LinearVel: {linear} | AngVel: {angular} | error: {angle_error}")

            self.send_velocity(linear, angular)

            rate.sleep()

    def send_velocity(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

if __name__ == '__main__':
    try:
        VirtualLineFollower()
    except rospy.ROSInterruptException:
        pass