from typing import Optional
import random

from pteranodon import AbstractDrone
from pteranodon.implementations import VideoStreamGST
from pteranodon.networks import MobileNetV1


class RandomMove(AbstractDrone):
    """
    Randomly move the drone

    Assumed to be run from the root of the pteranodon repository
    """

    def __init__(self, mobilenet_path: Optional[str] = None):
        self._cam = VideoStreamGST()
        if mobilenet_path is None:
            self._nn = MobileNetV1()
        else:
            self._nn = MobileNetV1(mobilenet_path)
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
            self.maneuver_to(
                random.randrange(-1, 1),
                random.randrange(-1, 1),
                random.randrange(-1, 1),
            )  # call some random relative maneuver

    def teardown(self):
        """Teardown drone"""
        self.sensor.stop_all_sensors()
