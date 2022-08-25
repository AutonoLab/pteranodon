from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, TypeVar, Type

from mavsdk import System

from .abstract_plugin import AbstractPlugin
from .base_plugins.abstract_base_plugin import AbstractBasePlugin
from .ext_plugins.abstract_custom_plugin import AbstractCustomPlugin
from .base_plugins import *
from .ext_plugins import *


class PluginManager:
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger, ext_args: Dict, custom_args: Dict)\
            -> None:
        self._system = system
        self._loop = loop
        self._logger = logger
        self._ext_args = ext_args
        self._custom_args = custom_args

        plugins = [Action, Calibration, Core, FollowMe, Geofence, Info, Offboard, Param, Telemetry, Transponder]
        self._base_plugins = {}
        for plugin in plugins:
            plugin = plugin(self._system, self._loop, self._logger)
            self._base_plugins[plugin.name] = plugin

        plugins = [Sensor, Relative]
        self._ext_plugins = {}
        for plugin in plugins:
            plugin = plugin(self._system, self._loop, self._logger, self._base_plugins, self._ext_args)
            self._ext_plugins[plugin.name] = plugin

        self._custom_plugins : Dict[str, AbstractCustomPlugin] = {}

    @property
    def base_plugins(self) -> Dict:
        return self._base_plugins

    @property
    def ext_plugins(self) -> Dict:
        return self._ext_plugins

    @property
    def custom_plugins(self) -> Dict:
        return self._custom_plugins

    T = TypeVar('T', bound=AbstractCustomPlugin)

    def add_plugin(self, new_plugin_class: Type[T]) -> None:
        new_plugin = new_plugin_class(self._system, self._loop, self._logger, self._base_plugins, self._custom_args)
        self._custom_plugins[new_plugin.name] = new_plugin
