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
	self.recon = Reconstruct()
	self.recon.listen.start()
    
    def get_frames(self):
        rclpy.spin(self.subscriber)
	if self.recon.data is not None:
		_, data = loads(self.recon.data)
		image = cv2.imdecode(data, flags=cv2.IMREAD_COLOR)
		return image
    
    def update_data(self):
        """Update sensor data"""
        self.data.update(self.get_frames())

    def teardown(self):
	# destroy subscriber node, shutdown ros client context, terminate udp listening thread
        self.subscriber.destroy_node()
        rclpy.shutdown()
	self.recon.terminate()
	self.recon.listen.join()
        

class SubNode(Node):
	def __init__(self):
		super().__init__("subscriber")
		self.subscriber = self.create_subscription(
			Image, 
			"/demo_cam/mycamera/depth_demo", #path to gazebo topic
            		self.callback, 
            		10)
		self.frag = Fragment()
		
	def callback(self, msg):
		try:
			encoding = msg.encoding
			cv_image = CvBridge().imgmsg_to_cv2(msg, encoding)
			cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
			cv_image = cv2.encode(".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
			string_data = dumps(cv_image)
			self.frag.fragment(string_data)
		except CvBridgeError as e:
			print(e)


class Fragment():
	"""fragment data into smaller datagrams, and send them via udp port"""
	def __init__(self, ip="127.0.0.1", port=5005, packsize=100000):
		self.ip = ip
		self.port = port
		self.packsize = packsize
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.packsize)

	def fragment(self, data):
		framesize = getsizeof(data)
		# split the frame into datagrams
		for x in range( framesize // self.packsize + 1 ):
			self.sock.sendto(data[x*self.packsize : x*self.packsize+self.packsize], (self.ip, self.port))
	
	
class Reconstruct():
	"""listen for messages over udp port and reconstruct them back into the original images"""
	def __init__(self, ip="127.0.0.1", port=5005, packsize=100000):
		self.msgList = []
		self.ip = ip
		self.port = port
		self.packsize = packsize
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind((self.ip, self.port))
		self.listen = threading.Thread(target=self.listener)
		self.data = None
		self.cont_listen = True

	def listener(self):
		while self.cont_listen is True:
			self.data, _ = self.sock.recvfrom(self.packsize)

	def reconstruct(self):
		while True:
			if self.data is not None:
				_, data = loads(self.data)
				image = cv2.imdecode(data, flags=cv2.IMREAD_COLOR)
				cv2.imshow("image", image)
				cv2.waitKey(30)
				
	def terminate(self):
		self.cont_listen = False
		
