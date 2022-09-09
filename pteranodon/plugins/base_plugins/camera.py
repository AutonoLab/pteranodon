import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List, Optional

from mavsdk import System, camera

from .abstract_base_plugin import AbstractBasePlugin

# TODO: Methods to implement
'''
current_settings
get_setting
information
list_photos
mode
possible_setting_options
prepare
select_camera
set_mode
set_setting
start_photo_interval
start_video
start_video_streaming
status
stop_photo_interval
stop_video
stop_video_streaming
take_photo
video_stream_info
'''

class Camera(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("camera", system, loop, logger)

        self._capture_info = None

        self._capture_info_task = asyncio.ensure_future(self._update_capture_info(), loop=self._loop)

    async def _update_capture_info(self) -> None:
        async for info in self._system.camera.capture_info():
            if info != self._capture_info:
                self._capture_info = info

    def format_storage(self) -> None:
        """
        Formats the storage (e.g. SD card) in the camera.

        This will delete all content of the camera storage!
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.format_storage(), loop=self._loop)
        )

    @property
    def capture_info(self) -> Optional[camera.CaptureInfo]:
        """
        :return: The existing capture info
        :rtype: CaptureInfo
        """
        return self._capture_info


