from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from ..abstract_meta_plugin import AbstractMetaPlugin
from ...base_plugins.param import Param
from ...base_plugins.telemetry import Telemetry


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
        super().__init__("config", system, loop, logger, base_plugins, ext_args)

        self._end_init()
