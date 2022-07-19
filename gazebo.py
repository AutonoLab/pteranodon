import time

from drone import Drone
from VideoStreamGST import Video
from hlca import FrameProcessor


# Concrete implemention of DroneInterface using HexSoon edu 450
class Gazebo(Drone):
    def __init__(self, min_follow_dist=5.0, time_slice=0.05) -> None:
        print("creating camera...")
        self.cam = Video()
        self.frame = None
        print("creating hlca instance...")
        self.fp = FrameProcessor(cnn_score_min=0.90, output_path="algo_output.mp4", save_output=True)

        print("running Drone.__init__ ...")
        super().__init__(address="udp://:14540", time_slice=time_slice, min_follow_distance=min_follow_dist)

        print("done init")

    def setup(self):
        self.frame = self.cam.frame()
        if self.frame is None:
            print("Frame is given as none, retrying until success. Disconnect QGroundControl video")
        while self.frame is None:
            self.frame = self.cam.frame()
        _ = self.fp.processFrame(self.frame, display=True)

    def loop(self):
        self.frame = self.cam.frame()
        _ = self.fp.processFrame(self.frame, display=True)

        # if motion_vector is not None:
        #     x, y = motion_vector
        #     cam_point = self.cam.deprojectPixelToPoint(frame=self.frame, cnn_x=x, cnn_y=y)
        #     # transform the cam_point to the drone_point
        #     front, right, down = cam_point[2], cam_point[0], 0 - cam_point[1]  # I think these points also need to be offset from the current location of the drone
        #     self.maneuver_to(front, right, down)

    def teardown(self):
        self.fp.close()


if __name__ == "__main__":
    drone = Gazebo()
    time.sleep(5)

    drone.arm()
    drone.takeoff()
    time.sleep(5)
    drone.start_offboard()
    drone.maneuver_to(front=10.0, right=10.0, down=10.0)

    key = input("press l to start autonomous flight loop, press any other key to stop flight")
    if key == "l":
        drone.start_loop()
        _ = input("press any key to end autonomous flight")

    drone.land()
    drone.stop()
