import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import List, Optional, Union

from functools import partial

from mavsdk import System, camera

from .abstract_base_plugin import AbstractBasePlugin

# TODO: Methods to implement
'''
list_photos
'''


class Camera(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("camera", system, loop, logger)

        self._current_camera_id: Optional[int] = None

        self._capture_info : Optional[camera.CaptureInfo] = None
        self._information : Optional[camera.Information] = None
        self._mode : Optional[camera.Mode] = None
        self._status : Optional[camera.Status] = None
        self._video_stream_info : Optional[camera.VideoStreamInfo] = None
        self._current_settings : List[camera.Setting] = []
        self._possible_setting_options: List[camera.SettingOptions] = []


        # Tasks of subscribed properties
        self._capture_info_task = asyncio.ensure_future(self._update_capture_info(), loop=self._loop)
        self._information_task = asyncio.ensure_future(self._update_information(), loop=self._loop)
        self._mode_task = asyncio.ensure_future(self._update_mode(), loop=self._loop)
        self._status_task = asyncio.ensure_future(self._update_status(), loop=self._loop)
        self._video_stream_info_task = asyncio.ensure_future(self._update_vstream_info(), loop=self._loop)

        # Only want to fetch the current settings and options once on init
        super().submit_task(
            asyncio.ensure_future(self._update_current_settings(), loop=self._loop)
        )
        super().submit_task(
            asyncio.ensure_future(self._update_possible_setting_opts(), loop=self._loop)
        )

        self._system.camera.possible_setting_options()

    def prepare(self) -> None:
        """
        Prepare the camera plugin (e.g. download the camera definition, etc)
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.prepare(), loop=self._loop)
        )

    def format_storage(self) -> None:
        """
        Formats the storage (e.g. SD card) in the camera.

        This will delete all content of the camera storage!
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.format_storage(), loop=self._loop)
        )

    def start_photo_interval(self, interval_s : float) -> None:
        """
        Start photo timelapse with a given interval

        :param interval_s: Interval between photos (in seconds)
        :type interval_s: float
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.start_photo_interval(interval_s), loop=self._loop)
        )
        self._status.photo_interval_on = True

    def stop_photo_interval(self) -> None:
        """
        Stop a running photo timelapse
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.stop_photo_interval(), loop=self._loop)
        )
        self._status.photo_interval_on = False

    def start_video(self) -> None:
        """
        Start a video recording
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.start_video(), loop=self._loop)
        )
        self._status.video_on = True

    def stop_video(self) -> None:
        """
        Stop a running video recording
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.stop_video(), loop=self._loop)
        )
        self._status.video_on = False

    def start_video_streaming(self) -> None:
        """
        Start video streaming
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.start_video_streaming(), loop=self._loop)
        )

    def stop_video_streaming(self) -> None:
        """
        Stop current video streaming
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.stop_video_streaming(), loop=self._loop)
        )

    def take_photo(self) -> None:
        """
        Take one photo
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.take_photo(), loop=self._loop)
        )

    def select_camera(self, camera_id : int) -> None:
        """
        Select current camera.

        Bind the plugin instance to a specific camera_id

        :param camera_id: The ID of the camera to select
        :type camera_id: int32
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.select_camera(camera_id), loop=self._loop)
        )
        self._current_camera_id = camera_id

    def set_mode(self, mode : camera.Mode) -> None:
        """
        Set camera mode

        :param mode: Camera mode to set
        :type mode: camera.Mode
        """
        super().submit_task(
            asyncio.ensure_future(self._system.camera.set_mode(mode), loop=self._loop)
        )
        self._mode = mode

    def set_setting(self, setting : Union[camera.Setting, int], option : Optional[Union[camera.Option, int]] = None) -> None:
        """
        Set a setting to some value.

        Examples:
            `set_setting(some_setting)`

            `set_setting(setting_id, option_id)`

            `set_setting(setting_id, some_option)`

            `set_setting(setting, some_option)`

        :param setting: The setting_id or a Setting object that contains the setting_id.
        If option is not set, the option_id must be included in this Setting object.
        :type setting: camera.Setting or uint32
        :param option: The option_id or an Option object that contains the option_id.
        Must be set if the Setting object does not in include the option_id.
        :type option: camera.Option or uint32
        """
        setting_obj : camera.Setting = setting
        if isinstance(setting, int):
            # Setting ID was passed instead of Setting object
            setting_obj = camera.Setting(setting, "", None, False)

        if option is not None:
            option_obj: camera.Option = option
            if isinstance(option, int):
                # Option ID was passed instead of Option object
                option_obj = camera.Option(option, "")

            setting_obj.option = option_obj

        if setting_obj.setting_id is None:
            self._logger.error("Could not set setting! No setting ID provided in object or function!")
            return None

        if setting_obj.option is None:
            self._logger.error("Could not set setting with ID {}! No option provided in object or function!".format(
                setting_obj.setting_id
            ))
            return None

        super().submit_task(
            asyncio.ensure_future(self._system.camera.set_setting(setting_obj), loop=self._loop)
        )

        # Update local setting object
        for setting_idx in range(len(self._current_settings)):
            if self._current_settings[setting_idx].setting_id == setting_obj.setting_id:
                self._current_settings[setting_idx].option = setting_obj.option
                break

    def get_setting(self, setting : Union[camera.Setting, int]) -> Optional[camera.Setting]:
        """
        Fetches a setting for the given setting ID (either directly given or set in the Setting object)

        :param setting: The Setting object with the chosen or the setting ID
        :type setting: camera.Setting or uint32
        :return: The requested camera.Setting object if found, None otherwise.
        :rtype: Optional[camera.Setting]
        """
        setting_obj : camera.Setting = setting
        if isinstance(setting, int):
            setting_obj = camera.Setting(setting, "", None, False)

        if setting_obj.setting_id is None:
            self._logger.error("Could not get setting! No setting ID provided in object or function!")
            return None

        for cam_setting in self._current_settings:
            if cam_setting.setting_id == setting_obj.setting_id:
                return cam_setting

        return None

    @property
    def capture_info(self) -> Optional[camera.CaptureInfo]:
        """
        :return: The current capture information
        :rtype: Optional[camera.CaptureInfo]
        """
        return self._capture_info

    @property
    def information(self) -> Optional[camera.Information]:
        """
        :return: The current camera information
        :rtype: Optional[camera.Information]
        """
        return self._information

    @property
    def mode(self) -> Optional[camera.Mode]:
        """
        :return: The current camera mode
        :rtype: Optional[camera.Mode]
        """
        return self._mode

    @property
    def status(self) -> Optional[camera.Status]:
        """
        :return: The current camera status
        :rtype: Optional[camera.Status]
        """
        return self._status

    @property
    def video_stream_info(self) -> Optional[camera.VideoStreamInfo]:
        """
        :return: The current video stream information
        :rtype: Optional[camera.VideoStreamInfo]
        """
        return self._video_stream_info

    @property
    def possible_settings_options(self) -> List[camera.SettingOptions]:
        """
        :return: The list of settings that can be changed
        :rtype: List[camera.SettingOptions]
        """
        return self._possible_setting_options

    @property
    def current_settings(self) -> List[camera.Setting]:
        """
        :return: The list of current camera settings
        :rtype: List[camera.Setting]
        """
        return self._current_settings

    async def _update_capture_info(self) -> None:
        async for info in self._system.camera.capture_info():
            if info != self._capture_info:
                self._capture_info = info

    async def _update_information(self) -> None:
        async for info in self._system.camera.information():
            if info != self._information:
                self._information = info

    async def _update_mode(self) -> None:
        async for mode in self._system.camera.mode():
            if mode != self._mode:
                self._mode = mode

    async def _update_status(self) -> None:
        async for status in self._system.camera.status():
            if status != self._status:
                self._status = status

    async def _update_vstream_info(self) -> None:
        async for vstream_info in self._system.camera.video_stream_info():
            if vstream_info != self._video_stream_info:
                self._video_stream_info = vstream_info

    async def _update_current_settings(self) -> None:

        # If any of the settings do not have an option set (empty data), do update
        # If the size is different, then definitely update.
        should_update_settings = any(setting.option is None for setting in self._current_settings)
        async for settings in self._system.camera.current_settings():
            if (len(settings) != len(self._current_settings)) or should_update_settings:
                self._current_settings = settings
                should_update_settings = False

    async def _update_possible_setting_opts(self) -> None:

        async for setting_options in self._system.camera.possible_setting_options():
            if len(setting_options) != len(self._possible_setting_options):
                self._possible_setting_options = setting_options

                if len(self._current_settings) == 0:
                    # If current settings have not been fetched yet, fill in setting data with "None" option
                    #       in the case get_settings is called by the user before they are returned.
                    # This is overwritten when current_settings returns
                    self._current_settings = [
                        camera.Setting(options.setting_id, options.setting_description, None, options.is_range)
                        for options in self._possible_setting_options
                    ]



