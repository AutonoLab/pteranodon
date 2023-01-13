import pyrealsense2 as rs
import numpy as np

from ...plugins.extension_plugins.sensor import AbstractSensor


class RealSense(AbstractSensor):
    """Class implementation for a physical RealSense camera"""

    def __init__(self, width=640, height=480, fps=30):
        # Configure depth and color streams
        super().__init__("RealSense-D435")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(self.config)

    def get_frames(self):
        """Get frames from sensor"""
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

    def deproj_pixel_to_point(self, depth_frame, frame_width, frame_height, cx, cy):
        """Deproject pixel to point in 3D space"""
        # convert center offset pixels to absolute offset from (0,0)
        x = frame_width / 2 + cx
        y = frame_height / 2 + cy

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
        """Update sensor data"""
        self.data.update(self.get_frames())

    def teardown(self):
        """Teardown sensor"""
        self.pipeline.stop()
