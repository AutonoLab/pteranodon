import time

from pteranodon import AbstractDrone
from pteranodon.implementations import VideoStreamGST


class RandomMove(AbstractDrone):
    """Randomly move the drone"""

    def __init__(self):
        self._cam = VideoStreamGST()
        super().__init__("RandomMove", sensor={"cam": self._cam})

    def setup(self):
        """Setup drone"""
        pass

    def loop(self):
        """Randomly moves the drone every time it gets a frame"""
        got, frame = self._cam.frame()
        if got:
            self.maneuver_to(1, 1, -1)  # call some random relative maneuver

    def teardown(self):
        """Teardown drone"""
        pass
