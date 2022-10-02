from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Type, Union, List

from mavsdk import System

from .base_plugins import (
    ActionServer,
    Action,
    Calibration,
    CameraServer,
    Camera,
    ComponentInformationServer,
    ComponentInformation,
    Core,
    Failure,
    FollowMe,
    Ftp,
    Geofence,
    Gimbal,
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
from .ext_plugins import Sensor, Relative
from .ext_plugins.abstract_custom_plugin import AbstractCustomPlugin
from .base_plugins.abstract_base_plugin import AbstractBasePlugin


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
        self._ext_plugins: Dict[str, AbstractCustomPlugin] = {}
        self._custom_plugins: Dict[str, AbstractCustomPlugin] = {}

        base_plugin_types: List[Type[AbstractBasePlugin]] = [
            ActionServer,
            Action,
            Calibration,
            CameraServer,
            Camera,
            ComponentInformationServer,
            ComponentInformation,
            Core,
            Failure,
            FollowMe,
            Ftp,
            Geofence,
            Gimbal,
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
        ]
        ext_plugin_types: List[Type[AbstractCustomPlugin]] = [Sensor, Relative]

        for base_type in base_plugin_types:
            if base_type in [Param, ComponentInformation, ComponentInformationServer, CameraServer, Camera, Mission, MissionRaw, MissionRawServer]:
                continue
            else:
                pass
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
    def base_plugins(self) -> Dict:
        """
        :return: Dict ; Returns a dictionary of the base plugins
        """
        return self._base_plugins

    @property
    def ext_plugins(self) -> Dict:
        """
        :return: Dict ; Returns a dictionary of external plugins
        """
        return self._ext_plugins

    @property
    def custom_plugins(self) -> Dict:
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
            new_plugin_obj = new_plugin(self._system, self._loop, self._logger, self._base_plugins, self._custom_args)  # type: ignore
        else:
            new_plugin_obj = new_plugin

        if not self._test_valid_plugin_name(new_plugin_obj.name):
            self._custom_plugins[new_plugin_obj.name] = new_plugin_obj

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
        for plugin in self._base_plugins.values():
            plugin.cancel_futures()
        for plugin in self._ext_plugins.values():
            plugin.cancel_futures()
        for plugin in self._custom_plugins.values():
            plugin.cancel_futures()
