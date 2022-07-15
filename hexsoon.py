import time

from drone import Drone
from RealSense import RealSense
from hlca import FrameProcessor


# Concrete implemention of DroneInterface using HexSoon edu 450
class Hexsoon(Drone):
    def __init__(self, min_follow_dist=5.0, time_slice=0.05) -> None:
        print("creating camera...")
        self.cam = RealSense()
        self.frame = None
        print("creating hlca instance...")
        self.fp = FrameProcessor(cnn_score_min=0.90, output_path="algo_output.mp4", save_output=True)

        print("running Drone.__init__ ...")
        super().__init__(address="serial:///dev/ttyACM0", time_slice=time_slice, min_follow_distance=min_follow_dist)

        print("done init")

    def setup(self):
        pass

    def loop(self):
        self.frame = self.cam.get_data()
        motion_vector = self.fp.processFrame(self.frame, display=False)

        if motion_vector is not None:
            cam_point = self.cam.deprojectPixelToPoint(frame=self.frame, cnn_x=motion_vector[0], cnn_y=motion_vector[1])
            # transform the cam_point to the drone_point
            front, right, down = cam_point[2], cam_point[0], 0 - cam_point[1]
            self.maneuver_to(front, right, down)

    def teardown(self):
        try:
            self.cam.close()
        except RuntimeError:
            pass
        self.fp.close()


if __name__ == "__main__":
    drone = Hexsoon()

    drone.arm()
    drone.takeoff()
    time.sleep(5)

    key = input("press l to start autonomous flight loop, press any other key to stop flight")
    if key == "l":
        drone.start_loop()
        _ = input("press any key to end autonomous flight")

    drone.land()
    drone.stop()
