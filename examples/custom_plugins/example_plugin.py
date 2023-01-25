from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from pteranodon import AbstractCustomPlugin
from pteranodon.plugins.base_plugins import Telemetry, Param
from pteranodon.plugins.extension_plugins import Relative


class ExamplePlugin(AbstractCustomPlugin):
    """Example plugin for pteranodon"""

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__(
            "example", system, loop, logger, base_plugins, ext_plugins, ext_args
        )

        self._telemetry: Telemetry = self._base_plugins["telemetry"]
        self._param: Param = self._base_plugins["param"]
        self._relative: Relative = self._ext_plugins["relative"]

        self._end_init()

    def example(self):
        """Example on how to use base and extension plugins inside of custom plugins"""
        self._logger.debug(f"{self._telemetry.battery}")
        self._logger.debug(f"{self._param.get_param_int('NAV_RCL_ACT')}")

        self._relative.maneuver_to(0, 0, 0)
