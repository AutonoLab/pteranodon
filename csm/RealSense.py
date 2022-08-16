import pyrealsense2 as rs
import numpy as np
import cv2

from pteranodon import Sensor


class RealSense(Sensor):
    def __init__(self):
        # Configure depth and color streams
        super().__init__("RealSense-D435")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

    def get_frames(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # updates sensordata
        # sensor.update(data)
        return color_image, depth_image, color_frame, depth_frame

    def deproj_pixel_to_point(self, depth_frame, cnn_x, cnn_y):
        # convert center offset pixels to absolute offset from (0,0)
        # TODO: get info on motion vector
        x = 320 + cnn_x
        y = 240 + cnn_y

        # get focal length intrinsic
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # get depth at (x, y)
        depth = depth_frame.get_distance(x, y)

        # get real world point at (x, y)
        # from perspective of camera: +x right, +y down, +z forward
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
        return depth_point

    # implement the abstract methods
    def update_data(self):
        self.data.update(self.get_frames())

    def teardown(self):
        self.pipeline.stop()
