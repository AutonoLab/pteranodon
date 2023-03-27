import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import socket
import numpy as np
import sys
from pickle import dumps, loads
from sys import getsizeof
import threading

from ...plugins.extension_plugins.sensor import AbstractSensor

IP = "127.0.0.1"
PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class VirtualRealSense(AbstractSensor):
    """Class implementation for a virtual RealSense camera"""

    def __init__(self, width=640, height=480, fps=30):
        super().__init__("Virtual_RealSense")
        rclpy.init()
        self.subscriber = SubNode()
    
    def get_frames(self):
        rclpy.spin(self.subscriber)
    
    def update_data(self):
        """Update sensor data"""
        self.data.update(self.get_frames())

    def teardown(self):
        self.subscriber.destroy_node()
        rclpy.shutdown()
        

class SubNode(Node):
	def __init__(self):
		super().__init__("subscriber")
		self.subscriber = self.create_subscription(
			Image,
			"/demo_cam/mycamera/depth_demo", #path to gazebo topic
            self.callback, 
            10)
		
	def callback(self, msg):

		try:
			encoding = msg.encoding
			cv_image = CvBridge().imgmsg_to_cv2(msg, encoding)
			cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
		except CvBridgeError as e:
			print(e)
            
