from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Type, Union, List

from mavsdk import System

from .base_plugins import (
    Action,
    Calibration,
    Core,
    FollowMe,
    Geofence,
    Info,
    Offboard,
    Param,
    Telemetry,
    Transponder,
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
            Action,
            Calibration,
            Core,
            FollowMe,
            Geofence,
            Info,
            Offboard,
            Param,
            Telemetry,
            Transponder,
        ]
        ext_plugin_types: List[Type[AbstractCustomPlugin]] = [Sensor, Relative]

        for base_type in base_plugin_types:
            base_plugin = base_type(self._system, self._loop, self._logger)  # type: ignore
            self._base_plugins[base_plugin.name] = base_plugin

        for ext_type in ext_plugin_types:
            ext_plugin = ext_type(self._system, self._loop, self._logger, self._base_plugins, self._ext_args)  # type: ignore
            self._ext_plugins[ext_plugin.name] = ext_plugin

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

        if new_plugin_obj.name in self._custom_plugins:
            self._logger.error(
                f'Could not add plugin with name "{new_plugin_obj.name}"! A plugin with that name already exists!'
            )
            return

        self._custom_plugins[new_plugin_obj.name] = new_plugin_obj
