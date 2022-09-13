import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from .abstract_base_plugin import AbstractBasePlugin

from mavsdk.telemetry_server import Battery

class TelemetryServer(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("telemetry_server", system, loop, logger)

    def publish_battery(self, battery: Battery) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_battery(battery), loop=self._loop)
        )

