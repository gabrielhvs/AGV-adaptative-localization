#!/usr/bin/env python3
import rospy
import yaml
import math
import os
import sys
import csv
import time
import statistics
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from tf.transformations import euler_from_quaternion

from PID_controller.motion import PIDController
class VirtualLineFollower:
    def __init__(self):

        rospy.init_node('virtual_line_follower')

        online = rospy.get_param('~online', '')

        if(online):
            rospy.Subscriber("/waypoints/poses", PoseArray, self.pose_array_callback, queue_size=1)
        else:
            self.load_line_points()

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.current_index = 0
        self.line_points = []

        self.distance_threshold = 0.05
        self.angle_threshold = 0.05
        self.linear_speed = 0.4
        self.angular_speed_gain = 1.5
        self.max_angular_speed = 0.4
        self.girando_na_quina = False  # NOVA VARIÁVEL DE ESTADO

        ### CSV logs
        self.distance_errors = []
        self.heading_errors = []


        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(script_dir,  "logs")
        os.makedirs(log_dir, exist_ok=True)
        metri_dir = os.path.join(script_dir,  "metrics")
        os.makedirs(metri_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, str(time.time()) + "_log.csv")
        self.metrics_path = os.path.join(metri_dir, str(time.time()) + "_metrics.csv")

        self.start_time = time.time()
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "timestamp", "goal_x", "goal_y", "robot_x", "robot_y", "linear_error", "angular_error"
        ])

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
        for pose in msg.poses:
            _, _, yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            points.append([pose.position.x, pose.position.y, yaw])
        if len(self.line_points) == 0:
            self.line_points = points
            self.control_loop()

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        self.last_time = None # ADICIONE ISTO para rastrear o tempo anterior
        angle_control = PIDController(0.05, 0.0, 0.4) 
        rate = rospy.Rate(100)
        step = False

        while not rospy.is_shutdown():
            if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
                rate.sleep()
                continue

            if self.current_index >= len(self.line_points):
                rospy.loginfo("Line completed. Stopping robot.")
                self.send_velocity(0, 0)
                break

            # 1. Calcule o intervalo de tempo (dt) real
            current_time = rospy.Time.now().to_sec()
            
            if self.last_time is None:
                dt = 0.033  # Valor seguro para o primeiro ciclo (assumindo ~30Hz)
            else:
                dt = current_time - self.last_time
                
            self.last_time = current_time # Salva o tempo para o ciclo seguinte

            goal_x, goal_y, goal_yaw = self.line_points[self.current_index]

            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)
            final_yaw_error = self.normalize_angle(goal_yaw - self.robot_yaw)

            # Log erros
            self.distance_errors.append(distance)
            self.heading_errors.append(angle_error)

            # Log CSV
            self.log_writer.writerow([
                time.time(), goal_x, goal_y, self.robot_x, self.robot_y, distance, angle_error
            ])

            forceBreak = False

            if distance > 0.11 and not(step):
                angular = angle_error
            elif abs(final_yaw_error) < 0.05:
                rospy.loginfo(f"Reached waypoint {self.current_index + 1}")
                self.current_index += 1
                step = False
                continue
            else:
                forceBreak = True
                angular = final_yaw_error
                step = True


            rospy.loginfo(f"Error Ativo {angular:.2f} | Error Yaw {final_yaw_error:.2f} | Dist {distance:.2f} | Estado Giro: {self.girando_na_quina}")
            
            # Passa o erro selecionado para o controlador
            angle_control.move(angular, dt, forceBreak)

            rate.sleep()

        ### Fim da missão: salvar métricas
        self.save_metrics()
        rospy.signal_shutdown('fim')

    def save_metrics(self):
        total_time = time.time() - self.start_time
        mean_error = sum(self.distance_errors) / len(self.distance_errors) if self.distance_errors else 0.0
        std_dev_dist = statistics.stdev(self.distance_errors) if len(self.distance_errors) > 1 else 0.0
        std_dev_heading = statistics.stdev(self.heading_errors) if len(self.heading_errors) > 1 else 0.0

        with open(self.metrics_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["total_time", "mean_linear_error", "std_dev_distance", "std_dev_heading"])
            writer.writerow([total_time, mean_error, std_dev_dist, std_dev_heading])

        rospy.loginfo(f"Métricas salvas em: {self.metrics_path}")
        self.log_file.close()

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