import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from mavsdk.failure import FailureUnit, FailureType

from .abstract_base_plugin import AbstractBasePlugin


class Failure(AbstractBasePlugin):
    """
    Inject failures into system to test failsafes.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("failure", system, loop, logger)

    def inject(
        self, failure_unit: FailureUnit, failure_type: FailureType, instance: int
    ) -> None:
        """
        Inject failures into system to test failsafes
        :param failure_unit: failure.FailureUnit ; The failure unit to send
        :param failure_type: failure.FailType ; the failure type to send
        :param instance: int ; instance to affect (0 for all)
        :return: None
        """
        super().submit_task(
            asyncio.ensure_future(
                self._system.failure.inject(failure_unit, failure_type, instance),
                loop=self._loop,
            )
        )
