#!/usr/bin/env python3
import rospy
import yaml
import math
import os
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

class VirtualLineFollower:
    def __init__(self):
        rospy.init_node('virtual_line_follower')
        
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
        
        self.distance_threshold = 0.4   
        self.linear_speed = 0.4                
        self.angular_speed_gain = 1.5
        self.max_angular_speed = 0.6           
        
        rospy.loginfo("Virtual Line Follower Node Initialized")
        self.control_loop()
    
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
            
            goal_x, goal_y = self.line_points[self.current_index]
            
            # Compute distance and angle to goal
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)
            

            if abs(angle_error) > 0.5:  # ~30 graus
                linear = self.linear_speed * 0.4
            else:
                linear = self.linear_speed
            
            # Parar se chegou no waypoint
            if distance <= self.distance_threshold:
                linear = 0.0
            
            
            angular = self.angular_speed_gain * angle_error
            angular = max(min(angular, self.max_angular_speed), -self.max_angular_speed)
            
            self.send_velocity(linear, angular)
            
            if distance <= self.distance_threshold:
                rospy.loginfo(f"Reached waypoint {self.current_index + 1}")
                self.current_index += 1
            
            rate.sleep()
    
    def send_velocity(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)
    
    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        VirtualLineFollower()
    except rospy.ROSInterruptException:
        pass