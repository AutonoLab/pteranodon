import time

from drone import Drone


class Gazebo(Drone):
    def __init__(self):
        super().__init__(address="udp://:14540")

    def setup(self):
        print("doing setup")

    def loop(self):
        print("looping")
        time.sleep(0.5)

    def teardown(self):
        print("doing teardown")

