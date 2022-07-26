import time

from easy_drone import AbstractDrone
from RealSense import RealSense
from hlca import FrameProcessor


# Concrete implemention of DroneInterface using HexSoon edu 450
class CSM_Hexsoon(AbstractDrone):
    def __init__(self, min_follow_dist=5.0, time_slice=0.05) -> None:
        print("creating camera...")
        self.cam = RealSense()
        self.frame = None
        print("creating hlca instance...")
        self.fp = FrameProcessor(cnn_score_min=0.90, output_path="algo_output.mp4", save_output=True)

        print("running Drone.__init__ ...")
        super().__init__(address="serial:///dev/ttyACM0",time_slice=time_slice, min_follow_distance=min_follow_dist)

        print("done init")

        self.put(self._drone.param.set_param_int, "COM_ARM_WO_GPS", 0)

    def setup(self):
        self.frame, depth_image, color_frame, depth_frame = self.cam.get_data()
        _ = self.fp.processFrame(self.frame, display=False)

    def loop(self):
        self.frame, depth_image, color_frame, depth_frame = self.cam.get_data()
        motion_vector = self.fp.processFrame(self.frame, display=False)

        if motion_vector is not None:
            print("acquired a motion vector")
            x, y = motion_vector
            cam_point = self.cam.deprojectPixelToPoint(depth_frame, cnn_x=x, cnn_y=y)
            # transform the cam_point to the drone_point
            front, right, down = cam_point[2], cam_point[0], 0 - cam_point[1]
            print(f"DISPATCHING TO: {front}, {right}, {down}")
            self.maneuver_to(front, right, down)

    def teardown(self):
        try:
            self.cam.close()
        except RuntimeError:
            pass
        self.fp.close()


if __name__ == "__main__":
    drone = CSM_Hexsoon()

    drone.arm()
    drone.takeoff()

    key = input("press l to start autonomous flight loop, press any other key to stop flight")
    if key == "l":
        drone.start_loop()
        _ = input("press any key to end autonomous flight")

    drone.return_to_launch()

    # TODO, I think there needs to be an additional method present which will wait on returning until the drone has landed
    # I supppose could steal the observe_in_air method from the MAVSDK examples

    drone.stop()
