#!/usr/bin/env python3
import rospy
import csv
import sys
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

def read_csv_waypoints(file_path):
    data = np.loadtxt(file_path, delimiter=',')
    if data.ndim == 1:
        data = np.expand_dims(data, axis=0)
    return data  # shape: [N, 2 or 3]

def create_pose(x, y, yaw=0.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, yaw)
    pose.orientation = Quaternion(*q)
    return pose

def create_marker(index, pose):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "waypoints"
    marker.id = index
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    return marker

def publish_waypoints(file_path):
    rospy.init_node("waypoint_publisher")

    pub_poses = rospy.Publisher("/waypoints/poses", PoseArray, queue_size=1, latch=True)
    pub_markers = rospy.Publisher("/waypoints/markers", MarkerArray, queue_size=1, latch=True)

    rate = rospy.Rate(1)  # Publish once
    rospy.sleep(1.0)

    try:
        waypoints = read_csv_waypoints(file_path)
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        marker_array = MarkerArray()

        for i in range(len(waypoints) - 1):
            x, y = waypoints[i, 0], waypoints[i, 1]
            import math

            dx = waypoints[i + 1, 0] - x
            dy = waypoints[i + 1, 1] - y
            yaw =  math.atan2(dy, dx)  # em radianos

            pose = create_pose(x, y, yaw)
            pose_array.poses.append(pose)
            marker = create_marker(i, pose)
            marker_array.markers.append(marker)

        pub_poses.publish(pose_array)
        pub_markers.publish(marker_array)
        rospy.loginfo(f"✅ Published {len(waypoints)} waypoints.")
    except Exception as e:
        rospy.logerr(f"❌ Error reading or publishing waypoints: {e}")

    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: waypoint_publisher.py path_to_waypoints.csv")
        sys.exit(1)
    file_path = sys.argv[1]
    publish_waypoints(file_path)
