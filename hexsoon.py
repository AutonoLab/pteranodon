import time

from drone import Drone
from RealSense import RealSense


# Concrete implemention of DroneInterface using HexSoon edu 450
class Hexsoon(Drone):
    def __init__(self, min_follow_dist=5.0, time_slice=0.05) -> None:
        super().__init__(address="serial:///dev/ttyACM0", time_slice=time_slice, min_follow_distance=min_follow_dist)
        self.cam = RealSense()
        self.frame = self.cam.get_data()

    def setup(self):
        self.frame = self.cam.get_data()

    def loop(self):
        pass

    def teardown(self):
        self.cam.close()


if __name__ == "__main__":
    drone = Hexsoon()

    drone.arm()
    drone.takeoff()

    time.sleep(10)

    drone.land()
    drone.stop()
