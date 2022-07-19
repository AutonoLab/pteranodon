from time import sleep

from drone import Drone

class Simple(Drone):
    def __init__(self, address="serial:///dev/ttyACM0"):
        super().__init__(address=address)

    def setup(self):
        pass

    def loop(self):
        time.sleep(1)

    def teardown(self):
        pass


if __name__ == "__main__":
    drone = Simple()
    drone.arm()
    drone.takeoff()

    time.sleep(10)

    drone.land()
    drone.stop()
