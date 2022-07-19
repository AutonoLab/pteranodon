import pyrealsense2 as rs
import numpy as np
import cv2

from Interfaces.CameraInterface import CameraInterface

from Sensor import Sensor
from SensorData import SensorData


class RealSense(CameraInterface):
    def __init__(self):
        # Configure depth and color streams
        super().__init__()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == "RGB Camera":
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == "L500":
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # Start streaming
        profile = self.config.resolve(self.pipeline)
        self.pipeline.start(self.config)

    def get_data(self):
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

    def deprojectPixelToPoint(self, depth_frame, cnn_x, cnn_y):
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

    def close(self):
        self.pipeline.stop()

    def getFrame(self):
        # Wait for a coherent pair of frames: depth and color
        return self.pipeline.wait_for_frames()


if __name__ == "__main__":
    sensor = Sensor("RealSenseD435")
    rsc = RealSense()
    frame = rsc.get_data()
    while True:
        cv2.imshow("camera_output", frame)
        if cv2.waitKey(1) != -1:
            rsc.close()
            break
        frame = rsc.get_data()
