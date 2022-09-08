import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List, Optional

from mavsdk import System, camera

from .abstract_base_plugin import AbstractBasePlugin



class Camera(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("camera", system, loop, logger)

        self._capture_info = None

        self._capture_info_task = asyncio.ensure_future(self._update_capture_info(), loop=self._loop)

    async def _update_capture_info(self) -> None:
        async for info in self._system.camera.capture_info():
            if info != self._capture_info:
                self._capture_info = info

    @property
    def capture_info(self) -> Optional[camera.CaptureInfo]:
        """
        :return: The existing capture info
        :rtype: CaptureInfo
        """
        return self._capture_info

