import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from time import sleep
from typing import List, Dict, Any, Callable, Tuple
from functools import partial

from mavsdk import System, camera_server

from .abstract_base_plugin import AbstractBasePlugin


class CameraServer(AbstractBasePlugin):

    # When a photo request is recieved, a photo must be returned. This function is defined by the end user
    #   and allows them to add their custom behavior
    PhotoRequestCallback = Callable[[int], Tuple[camera_server.TakePhotoFeedback, camera_server.CaptureInfo]]

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger,
                 cam_info : camera_server.Information, photo_request_callback : PhotoRequestCallback) -> None:
        super().__init__("camera_server", system, loop, logger)

        # Must be called as soon as the camera server is created
        self.set_information(cam_info)
        self._photo_request_callback = photo_request_callback

        self._take_photo_sub_task = asyncio.ensure_future(self._take_photo(), loop=self._loop)


    async def _take_photo(self) -> None:

        async for capture_req_idx in self._system.camera_server.take_photo():
            self._logger.info("Received image capture request with index {}".format(capture_req_idx))

            # Uses the photo request callback to get the arguments for the response_take_photo method.
            feedback_capture_tuple = self._photo_request_callback(capture_req_idx)
            self._respond_take_photo(*feedback_capture_tuple)


    def _respond_take_photo(self, take_photo_feedback : camera_server.TakePhotoFeedback, capture_info : camera_server.CaptureInfo):
        """
        Respond to an image capture request from SubscribeTakePhoto (_take_photo).

        :param take_photo_feedback: The feedback
        :type take_photo_feedback: camera_server.TakePhotoFeedback
        :param capture_info: The capture information
        :type capture_info: camera_server.CaptureInfo
        """
        super().submit_task(
            asyncio.ensure_future(
                self._system.camera_server.respond_take_photo(take_photo_feedback, capture_info),
                loop=self._loop
            )
        )

    def set_information(self, cam_info : camera_server.Information) -> None:
        """
        Sets the camera information. This must be called as soon as the camera server is created.

        :param cam_info: Information about the camera
        :type cam_info: camera.Information
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera_server.set_information(cam_info), loop=self._loop)
        )