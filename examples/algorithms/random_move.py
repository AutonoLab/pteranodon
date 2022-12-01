import time

from pteranodon import AbstractDrone
from pteranodon.implementations import VideoStreamGST
from pteranodon.networks import MobileNetV1


class RandomMove(AbstractDrone):
    """Randomly move the drone"""

    def __init__(self):
        self._cam = VideoStreamGST()
        self._nn = MobileNetV1()
        self._nn.init()  # here instead of setup due to threading stuff
        super().__init__("RandomMove", sensor={"cam": self._cam})

    def setup(self):
        """Setup drone"""
        self.sensor.start_all_sensors()

    def loop(self):
        """Randomly moves the drone every time it gets a frame"""
        got, frame = self._cam.data.value
        success, vector = self._nn.run(frame)
        if got:
            self.maneuver_to(1, 1, -1)  # call some random relative maneuver

    def teardown(self):
        """Teardown drone"""
        self.sensor.stop_all_sensors()
