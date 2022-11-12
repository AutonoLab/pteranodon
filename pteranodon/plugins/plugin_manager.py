from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Type, Union, List, Tuple

from mavsdk import System

from .abstract_plugin import AbstractPlugin
from .base_plugins.abstract_base_plugin import AbstractBasePlugin
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
from .extension_plugins.abstract_extension_plugin import AbstractExtensionPlugin
from .extension_plugins import Sensor, Relative#, Power
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
    ) -> None:
        self._system = system
        self._loop = loop
        self._logger = logger
        self._ext_args = ext_args

        self._base_plugins: Dict[str, AbstractBasePlugin] = {}
        self._ext_plugins: Dict[str, AbstractExtensionPlugin] = {}
        self._custom_plugins: Dict[str, AbstractCustomPlugin] = {}

        # Relatively sorted by amount of data being requested during init
        # Lowest to the highest generators, Highest to the lowest bandwidth in __init__
        # TODO: Gimbal, CameraServer, ComponentInformation, ComponentInformationServer # pylint: disable=fixme
        base_plugin_types: List[Type[AbstractBasePlugin]] = sorted(
            [
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
            ],
            key=self._get_sort_keys,
            reverse=False,
        )
        ext_plugin_types: List[Type[AbstractExtensionPlugin]] = sorted(
            [Sensor, Relative],
            #[Sensor, Relative, Power],
            key=self._get_sort_keys,
            reverse=False,
        )

        for base_type in base_plugin_types:
            self._logger.info(f"Beginning setup of: {base_type} plugin")
            base_plugin = base_type(self._system, self._loop, self._logger)  # type: ignore
            if not base_plugin.ready:
                self._logger.error(
                    f"Plugin {base_plugin.name} is not ready after intialization!"
                )
                continue
            self._base_plugins[base_plugin.name] = base_plugin
            AbstractBasePlugin.register(base_type)

        for ext_type in ext_plugin_types:
            ext_plugin = ext_type(self._system, self._loop, self._logger, self._base_plugins, self._ext_args)  # type: ignore
            if not ext_plugin.ready:
                self._logger.error(
                    f"Plugin {ext_plugin.name} is not ready after intialization!"
                )
                continue
            self._ext_plugins[ext_plugin.name] = ext_plugin
            AbstractExtensionPlugin.register(ext_type)

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
            cust_args = (
                self._system,
                self._loop,
                self._logger,
                self._base_plugins,
                self._ext_plugins,
                self._ext_args,
            )
            new_plugin_obj = new_plugin(*cust_args)  # type: ignore
        else:
            new_plugin_obj = new_plugin

        if not self._test_valid_plugin_name(new_plugin_obj.name):
            self._custom_plugins[new_plugin_obj.name] = new_plugin_obj
            setattr(self, new_plugin_obj.name, new_plugin_obj)

        AbstractCustomPlugin.register(type(new_plugin_obj))

    def _test_valid_plugin_name(self, plugin_name: str) -> bool:
        if hasattr(self, plugin_name):
            self._logger.error(
                f'Could not add plugin with name "{plugin_name}"! A plugin with that name already exists!'
            )
            return True
        return False

    def _get_sort_keys(self, a: Type[AbstractPlugin]) -> Tuple[int, int]:
        num_gens = len(
            [
                func
                for func in dir(a)
                if not func.startswith("_") and "register" in func and "handler" in func
            ]
        )
        return (a.bandwidth(), num_gens)

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

    @property
    def action_server(self) -> ActionServer:
        """
        The ActionServer plugin instance
        """
        return self.base_plugins["action_server"]  # type: ignore

    @property
    def action(self) -> Action:
        """
        The Action plugin instance
        """
        return self.base_plugins["action"]  # type: ignore

    @property
    def calibration(self) -> Calibration:
        """
        The Calibration plugin instance
        """
        return self.base_plugins["calibration"]  # type: ignore

    @property
    def camera_server(self) -> CameraServer:
        """
        The CameraServer plugin instance
        """
        return self.base_plugins["camera_server"]  # type: ignore

    @property
    def camera(self) -> Camera:
        """
        The Camera plugin instance
        """
        return self.base_plugins["camera"]  # type: ignore

    @property
    def component_information_server(self) -> ComponentInformationServer:
        """
        The ComponentInformationServer plugin instance
        """
        return self.base_plugins["component_information_server"]  # type: ignore

    @property
    def component_information(self) -> ComponentInformation:
        """
        The ComponentInformation plugin instance
        """
        return self.base_plugins["component_information"]  # type: ignore

    @property
    def core(self) -> Core:
        """
        The Core plugin instance
        """
        return self.base_plugins["core"]  # type: ignore

    @property
    def failure(self) -> Failure:
        """
        The Failure plugin instance
        """
        return self.base_plugins["failure"]  # type: ignore

    @property
    def follow_me(self) -> FollowMe:
        """
        The FollowMe plugin instance
        """
        return self.base_plugins["follow_me"]  # type: ignore

    @property
    def ftp(self) -> Ftp:
        """
        The Ftp plugin instance
        """
        return self.base_plugins["ftp"]  # type: ignore

    @property
    def geofence(self) -> Geofence:
        """
        The Geofence plugin instance
        """
        return self.base_plugins["geofence"]  # type: ignore

    @property
    def gimbal(self) -> Gimbal:
        """
        The  plugin instance
        """
        return self.base_plugins["gimbal"]  # type: ignore

    @property
    def info(self) -> Info:
        """
        The Info plugin instance
        """
        return self.base_plugins["info"]  # type: ignore

    @property
    def log_files(self) -> LogFiles:
        """
        The LogFiles plugin instance
        """
        return self.base_plugins["log_files"]  # type: ignore

    @property
    def manual_control(self) -> ManualControl:
        """
        The ManualControl plugin instance
        """
        return self.base_plugins["manual_control"]  # type: ignore

    @property
    def mission_raw_server(self) -> MissionRawServer:
        """
        The MissionRawServer plugin instance
        """
        return self.base_plugins["mission_raw_server"]  # type: ignore

    @property
    def mission_raw(self) -> MissionRaw:
        """
        The MissionRaw plugin instance
        """
        return self.base_plugins["mission_raw"]  # type: ignore

    @property
    def mission(self) -> Mission:
        """
        The Mission plugin instance
        """
        return self.base_plugins["mission"]  # type: ignore

    @property
    def mocap(self) -> Mocap:
        """
        The Mocap plugin instance
        """
        return self.base_plugins["mocap"]  # type: ignore

    @property
    def offboard(self) -> Offboard:
        """
        The Offboard plugin instance
        """
        return self.base_plugins["offboard"]  # type: ignore

    @property
    def param_server(self) -> ParamServer:
        """
        The ParamServer plugin instance
        """
        return self.base_plugins["param_server"]  # type: ignore

    @property
    def param(self) -> Param:
        """
        The Param plugin instance
        """
        return self.base_plugins["param"]  # type: ignore

    @property
    def rtk(self) -> Rtk:
        """
        The Rtk plugin instance
        """
        return self.base_plugins["rtk"]  # type: ignore

    @property
    def server_utility(self) -> ServerUtility:
        """
        The ServerUtility plugin instance
        """
        return self.base_plugins["server_utility"]  # type: ignore

    @property
    def shell(self) -> Shell:
        """
        The Shell plugin instance
        """
        return self.base_plugins["shell"]  # type: ignore

    @property
    def telemetry_server(self) -> TelemetryServer:
        """
        The TelemetryServer plugin instance
        """
        return self.base_plugins["telemetry_server"]  # type: ignore

    @property
    def telemetry(self) -> Telemetry:
        """
        The Telemetry plugin instance
        """
        return self.base_plugins["telemetry"]  # type: ignore

    @property
    def tracking_server(self) -> TrackingServer:
        """
        The TrackingServer plugin instance
        """
        return self.base_plugins["tracking_server"]  # type: ignore

    @property
    def transponder(self) -> Transponder:
        """
        The Transponder plugin instance
        """
        return self.base_plugins["transponder"]  # type: ignore

    @property
    def tune(self) -> Tune:
        """
        The Tune plugin instance
        """
        return self.base_plugins["tune"]  # type: ignore

    @property
    def sensor(self) -> Sensor:
        """
        :return: The Sensor plugin class instance
        """
        return self.ext_plugins["sensor"]  # type: ignore

    @property
    def relative(self) -> Relative:
        """
        :return: The Relative plugin class instance
        """
        return self.ext_plugins["relative"]  # type: ignore

    # @property
    # def power(self) -> Power:
    #     """
    #     :return: The Power plugin class instance
    #     """
    #     return self.ext_plugins["power"]  # type: ignore
