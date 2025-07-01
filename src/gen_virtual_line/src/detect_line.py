#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from tf.transformations import quaternion_matrix
import numpy as np
import cv2

class DetectLineNode:
    def __init__(self):
        rospy.init_node("camera_ground_projector", log_level=rospy.FATAL)
        rospy.on_shutdown(self.shutdown_hook)

        self.K = None  # Aguardando câmera_info
        self.got_camera_info = False
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.frame_camera = rospy.get_param("~camera_frame", "zed_camera")
        self.frame_mapa = rospy.get_param("~map_frame", "map")
        self.path = []
        self.local_points = []
        self.cv2Image = []

        # Subscribers
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)

        # Publisher
        self.image_pub = rospy.Publisher("/camera/projected_ground", Image, queue_size=1)

        rospy.loginfo("Camera Ground Projection Node Initialized")


    def shutdown_hook(self):
        rospy.loginfo("ROS está encerrando (SIGINT recebido ou rosnode kill)")

    def calcular_centroide(self, pontos):
        if not pontos:
            return (0, 0)  # Retorna (0,0) para lista vazia

        soma_x = sum(ponto[0] for ponto in pontos)
        soma_y = sum(ponto[1] for ponto in pontos)
        n = len(pontos)
        return [soma_x / n, soma_y / n]

    def camera_info_callback(self, msg):
        if not self.got_camera_info:
            self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            self.got_camera_info = True
            rospy.loginfo("Camera intrinsics received.")

    def image_callback(self, msg):

        if not self.got_camera_info:
            rospy.logwarn_throttle(5, "Waiting for camera_info...")
            return

        try:
            self.cv2Image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        stamp = msg.header.stamp
        transform = self.tf_buffer.lookup_transform(self.frame_mapa, self.frame_camera, stamp)
        points = self.detect_line(self.cv2Image)
        point = self.calcular_centroide(points)
        self.local_points = self.projetar_para_mundo([point], transform)
        self.path.extend(self.local_points)


    def detect_line(self, image):
        h, w = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
        pontos = cv2.findNonZero(mask)
        pontos = [ i[0] for i in pontos]
        resume = []
        for i in range(0, len(pontos), int(len(pontos)/100)):
            resume.append(pontos[i])

        return resume


    def projetar_para_mundo(self, pontos_img, transform):
        R = quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])[:3, :3]
        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        K_inv = np.linalg.inv(self.K)
        poses = []
        for pt in pontos_img:  # reduz para não lotar
            pixel = np.array([pt[0], pt[1], 1.0])
            raio_camera = K_inv @ pixel
            dir_mundo = raio_camera
            escala = -t[2] / dir_mundo[2]
            ponto_global = t + R @ (escala * dir_mundo)
            poses.append(ponto_global)
        return poses