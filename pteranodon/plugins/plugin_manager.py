from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Type, Union, List

from mavsdk import System

from .ext_plugins.abstract_custom_plugin import AbstractCustomPlugin
from .base_plugins import *
from .base_plugins.abstract_base_plugin import AbstractBasePlugin
from .ext_plugins import *


class PluginManager:

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger, ext_args: Dict, custom_args: Dict)\
            -> None:
        self._system = system
        self._loop = loop
        self._logger = logger
        self._ext_args = ext_args
        self._custom_args = custom_args

        base_plugin_types: List[Type[AbstractBasePlugin]] = [Action, Calibration, Core, FollowMe,
                                                             Geofence, Info, Offboard, Param, Telemetry, Transponder]
        self._base_plugins = {}
        for plugin in base_plugin_types:
            plugin = plugin(self._system, self._loop, self._logger)  # type: ignore
            self._base_plugins[plugin.name] = plugin

        ext_plugin_types: List[Type[AbstractCustomPlugin]] = [Sensor, Relative]
        self._ext_plugins = {}
        for plugin in ext_plugin_types:
            plugin = plugin(self._system, self._loop, self._logger, self._base_plugins, self._ext_args)  # type: ignore
            self._ext_plugins[plugin.name] = plugin

        self._custom_plugins: Dict[str, AbstractCustomPlugin] = {}

    @property
    def base_plugins(self) -> Dict:
        return self._base_plugins

    @property
    def ext_plugins(self) -> Dict:
        return self._ext_plugins

    @property
    def custom_plugins(self) -> Dict:
        return self._custom_plugins

    def add_plugin(self, new_plugin: Union[AbstractCustomPlugin, Type[AbstractCustomPlugin]]) -> None:
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
            self._logger.error(f"Could not add plugin with name \"{new_plugin_obj.name}\"! A plugin with that name already exists!")
            return

        self._custom_plugins[new_plugin_obj.name] = new_plugin_obj
