import time

from pteranodon import AbstractDrone
from VideoStreamGST import Video
from hlca import FrameProcessor


# Concrete implemention of DroneInterface using HexSoon edu 450
class CSM_Gazebo(AbstractDrone):
    def __init__(self, min_follow_dist=5.0, time_slice=0.05) -> None:
        print("creating camera...")
        self.cam = Video()
        self.frame = None
        print("creating hlca instance...")
        self.fp = FrameProcessor(
            cnn_score_min=0.90, output_path="algo_output.mp4", save_output=True
        )
        self.cam.start()

        print("running Drone.__init__ ...")
        super().__init__(
            address="udp://:14540",
            time_slice=time_slice,
            min_follow_distance=min_follow_dist,
        )

        print("done init")

        self.put(self._drone.param.set_param_int, "COM_ARM_WO_GPS", 0)

    def setup(self):
        self.frame = self.cam.data.value
        if self.frame is None:
            print(
                "Frame is given as none, retrying until success. Disconnect QGroundControl video"
            )
        while self.frame is None:
            self.frame = self.cam.data.value
            time.sleep(0.1)
        _ = self.fp.processFrame(self.frame, display=False)

    def loop(self):
        self.frame = self.cam.data.value
        motion_vector = self.fp.processFrame(self.frame, display=True)

        if motion_vector is not None:
            x, y = motion_vector
            front = 5.0
            right = ((x - 320) / 320) * front
            down = ((y - 240) / 240) * front

            self.maneuver_to(front, right, down)

    def teardown(self):
        self.fp.close()


if __name__ == "__main__":
    drone = CSM_Gazebo()

    drone.arm()
    drone.takeoff()

    key = input(
        "press l to start autonomous flight loop, press any other key to stop flight"
    )
    if key == "l":
        drone.start_loop()
        _ = input("press any key to end autonomous flight")

    drone.return_to_launch()

    # TODO, I think there needs to be an additional method present which will wait on returning until the drone has landed
    # I supppose could steal the observe_in_air method from the MAVSDK examples

    drone.stop()
