from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from ..abstract_meta_plugin import AbstractMetaPlugin


class Ros(AbstractMetaPlugin):
    """Allows the user to use a ROS2 node for making calls to plugins."""

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__("ros", system, loop, logger, base_plugins, ext_plugins, ext_args)

        self._end_init()

    def start(self):
        """Starts the ROS2 node."""
        pass

    def stop(self):
        """Stops the ROS2 node."""
        pass
