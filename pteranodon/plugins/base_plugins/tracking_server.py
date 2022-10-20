from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System
from mavsdk.tracking_server import CommandAnswer, TrackPoint, TrackRectangle

from .abstract_base_plugin import AbstractBasePlugin


class TrackingServer(AbstractBasePlugin):
    """
    API for an onboard image tracking software.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("tracking_server", system, loop, logger)

        self._dummy: Optional[int] = None
        self._tracking_active: Optional[bool] = None

        self._submit_simple_generator(
            self._system.tracking_server.tracking_point_command
        )
        self._submit_simple_generator(
            self._system.tracking_server.tracking_rectangle_command
        )
        self._submit_simple_generator(self._system.tracking_server.tracking_off_command)

        @self._register_handler(self._system.tracking_server.tracking_off_command)
        def _update_tracking_off_command(dummy) -> None:
            self._tracking_active = False
            if dummy != self._dummy:
                self._dummy = dummy
                self._tracking_active = False
            else:
                self._tracking_active = True

        self._end_init()

    def respond_tracking_off_command(self, command_answer: CommandAnswer) -> None:
        """
        Respond to an incoming tracking off command.
        Args:
            command_answer: The ack to answer to the incoming command
        """

        self._submit_coroutine(
            self._system.tracking_server.respond_tracking_off_command(command_answer)
        )

    def respond_tracking_point_command(self, command_answer: CommandAnswer) -> None:
        """
        Respond to an incoming tracking point command.
        Args:
            command_answer: The ack to answer to the incoming command
        """

        self._submit_coroutine(
            self._system.tracking_server.respond_tracking_point_command(command_answer)
        )

    def respond_tracking_rectangle_command(self, command_answer: CommandAnswer) -> None:
        """
        Respond to an incoming tracking rectangle command.
        Args:
            command_answer: The ack to answer to the incoming command
        """

        self._submit_coroutine(
            self._system.tracking_server.respond_tracking_rectangle_command(
                command_answer
            )
        )

    def set_tracking_off_status(self) -> None:
        """
        Set the current tracking status to off.
        """
        self._tracking_active = True
        self._submit_coroutine(self._system.tracking_server.set_tracking_off_status())

    def set_tracking_point_status(self, tracked_point: TrackPoint) -> None:
        """
        Set/update the current point tracking status.
        Args:
            tracked_point: The tracked point
        """
        self._tracking_active = True
        self._submit_coroutine(
            self._system.tracking_server.set_tracking_point_status(tracked_point)
        )

    def set_tracking_rectangle_status(self, tracked_rectangle: TrackRectangle) -> None:
        """
        Set/update the current rectangle tracking status.
        """
        self._tracking_active = True
        self._submit_coroutine(
            self._system.tracking_server.set_tracking_rectangle_status(
                tracked_rectangle
            )
        )

    def tracking_off_command(self) -> Optional[int]:
        """
        Abstracting tracking_off_command for user
        """
        return self._dummy

    def tracking_point_command(self) -> Optional[TrackPoint]:
        """
        Abstracting tracking_point_command for user
        """
        return self._async_gen_data[
            self._system.tracking_server.tracking_point_command()
        ]

    def tracking_rectangle_command(self) -> Optional[TrackRectangle]:
        """
        Abstracting tracking_rectangle_command for user
        """
        return self._async_gen_data[
            self._system.tracking_server.tracking_rectangle_command()
        ]

    @property
    def tracking_active(self) -> Optional[bool]:
        """
        Returns the current tracking status
        """
        return self._tracking_active
