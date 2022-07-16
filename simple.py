import time

from drone import Drone


class Simple(Drone):
    def __init__(self, time_slice=0.05, min_follow_distance=5, bypass_com_rcl=True):
        super().__init__("udp://:14540", time_slice, min_follow_distance, bypass_com_rcl)

    def setup(self):
        pass

    def loop(self):
        pass

    def teardown(self):
        pass


if __name__ == "__main__":
    drone = Simple()

    drone.arm()

    drone.takeoff()

    time.sleep(5)

    drone.start_offboard()
    time.sleep(1)
    drone.stop_offboard()
    time.sleep(1)
    drone.land()
    drone.stop()    
    