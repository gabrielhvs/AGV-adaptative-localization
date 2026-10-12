#!/usr/bin/env python3

# ROS package (python)
import rospy
import csv
import os
import time
import math
import statistics
import sys 

# Messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist 

# Module imports
from PID_controller.motion import PIDController
from detector import LineDetector

class Follower:
    def __init__(self):
        self.detector = LineDetector()
        self.motion_planner = PIDController(0.05, 0.0, 0.4) 
        
        self.last_time = None 

        # --- CORNER DETECTION VARIABLES ---
        self.corner_count = 0
        self.is_turning = False
        self.total_corners = 11
        
        # Thresholds based on absolute angular error (|direction * 0.01|)
        self.curve_threshold = 0.5  
        self.straight_threshold = 0.15  
        
        # Publisher to stop the robot at the end of the trajectory
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # ----------------------------------

        rospy.init_node('line_follower')
        self.rate = rospy.Rate(30)

        self.subscriber = rospy.Subscriber('camera/image_raw', Image, self.camera_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Robot state
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # Error tracking
        self.distance_errors = []
        self.heading_errors = []

        # Create folders for logs
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(script_dir, "logs")
        metrics_dir = os.path.join(script_dir, "metrics")
        os.makedirs(log_dir, exist_ok=True)
        os.makedirs(metrics_dir, exist_ok=True)

        timestamp = str(int(time.time()))
        self.log_path = os.path.join(log_dir, f"{timestamp}_log.csv")
        self.metrics_path = os.path.join(metrics_dir, f"{timestamp}_metrics.csv")

        self.start_time = time.time()
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "timestamp", "robot_x", "robot_y", "linear_error", "angular_error"
        ])

        rospy.loginfo("Line Follower Initialized")

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.save_metrics()

    def write_video(self):
        for frame in self.detector.video_data:
            self.detector.video_writer.write(frame)

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def camera_callback(self, msg):
        current_time = rospy.Time.now().to_sec()
        
        if self.last_time is None:
            dt = 0.033  
        else:
            dt = current_time - self.last_time
            
        self.last_time = current_time 

        direction = self.detector.get_direction(message=msg, line_color='yellow', tol=15)
        
        # --- CORNER DETECTION LOGIC ---
        current_deviation = abs(direction * 0.01)
        
        if current_deviation > self.curve_threshold and not self.is_turning:
            self.corner_count += 1
            self.is_turning = True
            rospy.loginfo(f"Corner {self.corner_count} detected! (Deviation: {current_deviation:.2f})")
            
        elif current_deviation < self.straight_threshold and self.is_turning:
            self.is_turning = False
            rospy.loginfo("Turn completed, back on straight path.")
            
        if self.corner_count >= self.total_corners:
            rospy.loginfo("Polygon trajectory completed (5 corners). Shutting down node and simulation...")
            
            msg_stop = Twist()
            self.cmd_vel_pub.publish(msg_stop)
            
            rospy.signal_shutdown("Simulation finished by camera node")
            return 
        # ------------------------------
        
        self.motion_planner.move(-0.01 * direction, dt, False)

        if self.robot_x is not None and self.robot_y is not None:
            linear_error = abs(direction * 0.01)  
            angular_error = direction * 0.01  

            self.distance_errors.append(linear_error)
            self.heading_errors.append(angular_error)

            self.log_writer.writerow([
                time.time(), self.robot_x, self.robot_y, linear_error, angular_error
            ])

    def save_metrics(self):
        total_time = time.time() - self.start_time
        mean_error = sum(self.distance_errors) / len(self.distance_errors) if self.distance_errors else 0.0
        std_dev_dist = statistics.stdev(self.distance_errors) if len(self.distance_errors) > 1 else 0.0
        std_dev_heading = statistics.stdev(self.heading_errors) if len(self.heading_errors) > 1 else 0.0

        with open(self.metrics_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["total_time", "mean_linear_error", "std_dev_distance", "std_dev_heading"])
            writer.writerow([total_time, mean_error, std_dev_dist, std_dev_heading])

        rospy.loginfo(f"Metrics saved at: {self.metrics_path}")
        self.log_file.close()

        try:
            os.system("beep")
        except:
            print('\a')

if __name__ == '__main__':
    follower = Follower()
    follower.run()