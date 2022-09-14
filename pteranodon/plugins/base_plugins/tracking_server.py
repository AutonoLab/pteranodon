import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, tracking_server
from mavsdk.tracking_server import CommandAnswer, TrackPoint, TrackRectangle

from .abstract_base_plugin import AbstractBasePlugin


class TrackingServer(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("tracking_server", system, loop, logger)
        self._track_rectangle = None
        self._track_point = None
        self._dummy = None
        self._tracking_off_task = asyncio.ensure_future(self._update_tracking_off_command(), loop=self._loop)
        self._tracking_point_task = asyncio.ensure_future(self._update_tracking_point_command(), loop=self._loop)
        self._tracking_rectangle_task = asyncio.ensure_future(self._update_tracking_rectangle_command(),
                                                              loop=self._loop)

    def respond_tracking_off_command(self, command_answer: CommandAnswer) -> None:
        """
        Respond to an incoming tracking off command.
        Args:
            command_answer: The ack to answer to the incoming command
        """

        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.respond_tracking_off_command(command_answer),
                                  loop=self._loop)
        )

    def respond_tracking_point_command(self, command_answer: CommandAnswer) -> None:
        """
        Respond to an incoming tracking point command.
        Args:
            command_answer: The ack to answer to the incoming command
        """

        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.respond_tracking_point_command(command_answer),
                                  loop=self._loop)
        )

    def respond_tracking_rectangle_command(self, command_answer: CommandAnswer) -> None:
        """
        Respond to an incoming tracking rectangle command.
        Args:
            command_answer: The ack to answer to the incoming command
        """

        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.respond_tracking_rectangle_command(command_answer),
                                  loop=self._loop)
        )

    def set_tracking_off_status(self) -> None:
        """
        Set the current tracking status to off.
        """

        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.set_tracking_off_status(), loop=self._loop)
        )

    def set_tracking_point_status(self, tracked_point: TrackPoint) -> None:
        """
        Set/update the current point tracking status.
        Args:
            tracked_point: The tracked point
        """

        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.set_tracking_point_status(tracked_point),
                                  loop=self._loop)
        )

    def set_tracking_rectangle_status(self, tracked_rectangle: TrackRectangle) -> None:
        """
        Set/update the current rectangle tracking status.
        """

        super().submit_task(
            asyncio.ensure_future(self._system.tracking_server.set_tracking_rectangle_status(tracked_rectangle),
                                  loop=self._loop)
        )

    async def _update_tracking_off_command(self) -> None:
        """
        Subscribe to incoming tracking off command.
        """

        async for dummy in self._system.tracking_server.tracking_off_command():
            if dummy != self._dummy:
                self._dummy = dummy

    def tracking_off_command(self) -> int:
        """
        Abstracting tracking_off_command for user
        """

        return self._dummy

    async def _update_tracking_point_command(self) -> None:
        """
        Subscribe to incoming tracking point command.
        """

        async for track_point in self._system.tracking_server.tracking_point_command():
            if track_point != self._track_point:
                self._track_point = track_point

    def tracking_point_command(self) -> TrackPoint:
        """
        Abstracting tracking_point_command for user
        """

        return self._track_point

    async def _update_tracking_rectangle_command(self) -> None:
        """
        Subscribe to incoming tracking rectangle command.
        """

        async for track_rectangle in self._system.tracking_server.tracking_rectangle_command():
            if track_rectangle != self._track_rectangle:
                self._track_rectangle = track_rectangle

    def tracking_rectangle_command(self) -> TrackRectangle:
        """
        Abstracting tracking_rectangle_command for user
        """

        return self._track_rectangle


