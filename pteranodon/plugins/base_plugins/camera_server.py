from asyncio import AbstractEventLoop
from logging import Logger
from typing import Callable, Tuple, Optional

from mavsdk import System, camera_server
from mavsdk.camera_server import TakePhotoFeedback, CaptureInfo

from .abstract_base_plugin import AbstractBasePlugin


class CameraServer(AbstractBasePlugin):
    """
    Provides handling of camera trigger commands.
    """

    PhotoRequestCallbackType = Callable[[int], Tuple[TakePhotoFeedback, CaptureInfo]]

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
    ) -> None:
        super().__init__("camera_server", system, loop, logger)

        # Must be called as soon as the camera server is created
        self._cam_info: Optional[camera_server.Information] = None

        # Sets the request callback to the default since some behavior is required
        self._photo_request_callback: CameraServer.PhotoRequestCallbackType = (
            CameraServer._default_photo_request_callback
        )

        self._submit_simple_generator(self._system.camera_server.take_photo)
        self._register_handler(self._system.camera_server.take_photo)(self._take_photo)

        self._end_init()

    def _check_cam_info_set(self, action_name: str) -> bool:
        """
        Since the camera info must be set before any actions can be taken, this method checks for existence and
        prints an error message if it has not been set

        :param action_name: The action that is calling this function
        :type action_name: str
        :return: Whether the camera info has been set or not
        :rtype: bool
        """
        if self._cam_info is None:
            self._logger.error(
                f'Cannot execute action "{action_name}"! Camera info has not been set!'
            )
            return False
        return True

    def set_photo_request_callback(self, callback: PhotoRequestCallbackType):
        """
        If image capture requests must be processed in a way that is different from the default, pass that
        function in here

        :param callback: The function which returns the parameters to respond_take_photo
        :type callback: Callable[[int], Tuple[camera.TakePhotoFeedback, camera.CaptureInfo]]
        """
        # Even though this doesn't call anything, it really cements the importance
        if not self._check_cam_info_set("Set Photo Request Callback"):
            return

        self._photo_request_callback = callback

    @staticmethod
    def _default_photo_request_callback(
        index: int,
    ) -> Tuple[TakePhotoFeedback, CaptureInfo]:
        """
        The default implementation of the method which takes an index and returns CaptureInfo and feedback.
        Called when a `take_photo` request is received, result is passed to `respond_take_photo`

        :param index: take_photo index
        :type index: int32
        :return: The parameters of respond_take_photo, the feedback and the capture information
        :rtype: Tuple[TakePhotoFeedback, CaptureInfo]
        """
        position = camera_server.Position(0, 0, 0, 0)
        quat = camera_server.Quaternion(0, 0, 0, 0)

        capture_info = CaptureInfo(
            position=position,
            attitude_quaternion=quat,
            time_utc_us=0,
            is_success=False,
            index=index,
            file_url="/",
        )

        return TakePhotoFeedback.FAILED, capture_info

    def _take_photo(self) -> None:
        capture_req_idx_opt = self._async_gen_data[
            self._system.camera_server.take_photo()
        ]
        if capture_req_idx_opt is None:
            return
        capture_req_idx: int = capture_req_idx_opt
        self._logger.info(
            f"Received image capture request with index {capture_req_idx}"
        )

        # Uses the photo request callback to get the arguments for the response_take_photo method.
        self.set_in_progress(True)
        feedback_capture_tuple = self._photo_request_callback(capture_req_idx)
        self.set_in_progress(False)
        self._respond_take_photo(*feedback_capture_tuple)

    def set_in_progress(self, in_progress: bool) -> None:
        """
        Sets image capture in progress status flags. This should be set to true when the camera is busy taking a photo
        and false when it is done.

        :param in_progress: true if capture is in progress or false for idle.
        :type in_progress: bool
        """
        if not self._check_cam_info_set("Set In Progress"):
            return
        self._submit_coroutine(self._system.camera_server.set_in_progress(in_progress))

    def _respond_take_photo(
        self,
        take_photo_feedback: camera_server.TakePhotoFeedback,
        capture_info: camera_server.CaptureInfo,
    ):
        """
        Respond to an image capture request from SubscribeTakePhoto (_take_photo).

        :param take_photo_feedback: The feedback
        :type take_photo_feedback: camera_server.TakePhotoFeedback
        :param capture_info: The capture information
        :type capture_info: camera_server.CaptureInfo
        """
        self._submit_coroutine(
            self._system.camera_server.respond_take_photo(
                take_photo_feedback, capture_info
            )
        )

    def set_information(self, cam_info: camera_server.Information) -> None:
        """
        Sets the camera information. This must be called as soon as the camera server is created.

        :param cam_info: Information about the camera
        :type cam_info: camera.Information
        """

        self._cam_info = cam_info

        self._submit_coroutine(self._system.camera_server.set_information(cam_info))

    def register_take_photo_handler(self, handler: Callable) -> None:
        """
        Registers a function (Callable) to be a handler of the data stream
        :param handler: A Callable which gets executed each time new data is received
        """
        self._register_handler(self._system.camera_server.take_photo)(handler)
