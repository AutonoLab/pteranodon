from typing import Optional
import random

from pteranodon import AbstractDrone
from pteranodon.implementations.hardware import VideoStreamGST
from pteranodon.implementations.networks import MobileNetV1


class RandomMove(AbstractDrone):
    """
    Randomly move the drone

    Assumed to be run from the root of the pteranodon repository
    """

    def __init__(
        self,
        mobilenet_path: Optional[str] = None,
        config_path: Optional[str] = "examples/abstract_drones/random_move_config.json",
    ):
        self._cam = VideoStreamGST(port=5600)
        if mobilenet_path is None:
            self._nn = MobileNetV1()
        else:
            self._nn = MobileNetV1(mobilenet_path)
        self._nn.init()  # here instead of setup due to threading stuff

        super().__init__("RandomMove", sensors=[self._cam], config_file=config_path)

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
