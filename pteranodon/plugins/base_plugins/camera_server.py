import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from time import sleep
from typing import List, Dict, Any, Callable
from functools import partial

from mavsdk import System, camera

from .abstract_base_plugin import AbstractBasePlugin


class CameraServer(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger, cam_info : camera.Information) -> None:
        super().__init__("camera_server", system, loop, logger)

        # Must be called as soon as the camera server is created
        self.set_information(cam_info)



    def set_information(self, cam_info : camera.Information) -> None:
        """
        Sets the camera information. This must be called as soon as the camera server is created.

        :param cam_info: Information about the camera
        :type cam_info: camera.Information
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera_server.set_information(cam_info), loop=self._loop)
        )