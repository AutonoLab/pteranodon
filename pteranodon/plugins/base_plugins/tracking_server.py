import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from .abstract_base_plugin import AbstractBasePlugin

from mavsdk.tracking_server import CommandAnswer, TrackPoint, TrackRectangle


class TrackingServer(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("tracking_server", system, loop, logger)

    def respond_tracking_off_command(self, command_answer: CommandAnswer) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.respond_tracking_off_command(command_answer),
                                  loop=self._loop)
        )

    def respond_tracking_point_command(self, command_answer: CommandAnswer) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.respond_tracking_point_command(command_answer),
                                  loop=self._loop)
        )

    def respond_tracking_rectangle_command(self, command_answer: CommandAnswer) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.respond_tracking_rectangle_command(command_answer),
                                  loop=self._loop)
        )

    def set_tracking_off_status(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.set_tracking_off_status(), loop=self._loop)
        )

    def set_tracking_point_status(self, tracked_point: TrackPoint) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.set_tracking_point_status(tracked_point),
                                  loop=self._loop)
        )

    def set_tracking_rectangle_status(self, tracked_rectangle: TrackRectangle) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.set_tracking_rectangle_status(tracked_rectangle),
                                  loop=self._loop)
        )
