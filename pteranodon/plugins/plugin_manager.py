from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Type, Union, List

from mavsdk import System

from .base_plugins.abstract_base_plugin import AbstractBasePlugin
from .base_plugins import (
    ActionServer,
    Action,
    Calibration,
    # CameraServer,
    Camera,
    # ComponentInformationServer,
    # ComponentInformation,
    Core,
    Failure,
    FollowMe,
    Ftp,
    Geofence,
    # Gimbal,
    Info,
    LogFiles,
    ManualControl,
    MissionRawServer,
    MissionRaw,
    Mission,
    Mocap,
    Offboard,
    ParamServer,
    Param,
    Rtk,
    ServerUtility,
    Shell,
    TelemetryServer,
    Telemetry,
    TrackingServer,
    Transponder,
    Tune,
)
from .extension_plugins.abstract_extension_plugin import AbstractExtensionPlugin
from .extension_plugins import Sensor, Relative
from .custom_plugins import AbstractCustomPlugin


class PluginManager:
    """
    Manages plugins for use by the vehicle
    """

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        ext_args: Dict,
        custom_args: Dict,
    ) -> None:
        self._system = system
        self._loop = loop
        self._logger = logger
        self._ext_args = ext_args
        self._custom_args = custom_args

        self._base_plugins: Dict[str, AbstractBasePlugin] = {}
        self._ext_plugins: Dict[str, AbstractExtensionPlugin] = {}
        self._custom_plugins: Dict[str, AbstractCustomPlugin] = {}

        # Relatively sorted by amount of data being requested during init
        # Lowest to the highest generators, Highest to the lowest bandwidth in __init__
        # TODO: Gimbal, CameraServer, ComponentInformation, ComponentInformationServer # pylint: disable=fixme
        base_plugin_types: List[Type[AbstractBasePlugin]] = sorted(
            [
                Param,
                Mission,
                # Resume alphabetical ordering here
                ActionServer,
                Action,
                Calibration,
                # CameraServer,
                Camera,
                # ComponentInformationServer,
                # ComponentInformation,
                Core,
                Failure,
                FollowMe,
                Ftp,
                Geofence,
                # Gimbal,
                Info,
                LogFiles,
                ManualControl,
                MissionRawServer,
                MissionRaw,
                Mocap,
                Offboard,
                ParamServer,
                Rtk,
                ServerUtility,
                Shell,
                TelemetryServer,
                Telemetry,
                TrackingServer,
                Transponder,
                Tune,
            ],
            key=lambda a: a.num_generators,
            reverse=False,
        )
        ext_plugin_types: List[Type[AbstractExtensionPlugin]] = sorted(
            [Sensor, Relative], key=lambda a: a.num_generators, reverse=False
        )

        for base_type in base_plugin_types:
            self._logger.info(f"Beginning setup of: {base_type} plugin")
            base_plugin = base_type(self._system, self._loop, self._logger)  # type: ignore
            if not self._test_valid_plugin_name(base_plugin.name):
                self._base_plugins[base_plugin.name] = base_plugin
                setattr(self, base_plugin.name, base_plugin)

        for ext_type in ext_plugin_types:
            ext_plugin = ext_type(self._system, self._loop, self._logger, self._base_plugins, self._ext_args)  # type: ignore
            if not self._test_valid_plugin_name(ext_plugin.name):
                self._ext_plugins[ext_plugin.name] = ext_plugin
                setattr(self, ext_plugin.name, ext_plugin)

    @property
    def base_plugins(self) -> Dict[str, AbstractBasePlugin]:
        """
        :return: Dict ; Returns a dictionary of the base plugins
        """
        return self._base_plugins

    @property
    def ext_plugins(self) -> Dict[str, AbstractExtensionPlugin]:
        """
        :return: Dict ; Returns a dictionary of external plugins
        """
        return self._ext_plugins

    @property
    def custom_plugins(self) -> Dict[str, AbstractCustomPlugin]:
        """
        :return: Dict ; Returns a dictionary of custom plugins
        """
        return self._custom_plugins

    def add_plugin(
        self, new_plugin: Union[AbstractCustomPlugin, Type[AbstractCustomPlugin]]
    ) -> None:
        """
        Adds a custom plugin to the plugin manager

        :param new_plugin: Either the custom plugin class or the custom plugin instance to add
        :type new_plugin: AbstractCustomPlugin sub-class type or instance.
        """
        new_plugin_obj: AbstractCustomPlugin
        if isinstance(new_plugin, type):
            new_plugin_obj = new_plugin(self._system, self._loop, self._logger, self._base_plugins, self._ext_plugins, self._custom_args)  # type: ignore
        else:
            new_plugin_obj = new_plugin

        if not self._test_valid_plugin_name(new_plugin_obj.name):
            self._custom_plugins[new_plugin_obj.name] = new_plugin_obj
            setattr(self, new_plugin_obj.name, new_plugin_obj)

    def _test_valid_plugin_name(self, plugin_name: str) -> bool:
        if hasattr(self, plugin_name):
            self._logger.error(
                f'Could not add plugin with name "{plugin_name}"! A plugin with that name already exists!'
            )
            return True
        return False

    def cancel_all_futures(self) -> None:
        """
        Force cancels all running (or yet to run) Futures in ALL plugins
        """
        for base_plugin in self._base_plugins.values():
            base_plugin.cancel_futures()
        for ext_plugin in self._ext_plugins.values():
            ext_plugin.cancel_futures()
        for custom_plugin in self._custom_plugins.values():
            custom_plugin.cancel_futures()
