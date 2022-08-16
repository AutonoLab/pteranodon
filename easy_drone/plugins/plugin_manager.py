from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

import .base_plugins as base_plugins
from .abstract_plugin import AbstractPlugin
from .base_plugins.abstract_base_plugin import AbstractBasePlugin


class PluginManager:
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        self._system = system
        self._loop = loop
        self._logger = logger

        plugins = [plugin for plugin in dir(base_plugins) if not plugin.startswith("_")]
        plugins = [plugin for plugin in plugins if isinstance(getattr(base_plugins, plugin), AbstractBasePlugin)]

        self._plugins = {}
        for plugin in plugins:
            plugin = getattr(base_plugins, plugin)
            self._plugins[plugin.name] = plugin(self._system, self._loop, self._logger)

    @property
    def plugins(self) -> Dict:
        return self._plugins

    def add_plugin(self, new_plugin: AbstractBasePlugin) -> None:
        try:
            if self._plugins[new_plugin.name]:
                raise KeyError(f"Plugin {new_plugin.name} already exists")
            else:
                self._plugins[new_plugin.name] = new_plugin
        except Exception:
            new_plugin = new_plugin(self._system, self._loop, self._logger)
