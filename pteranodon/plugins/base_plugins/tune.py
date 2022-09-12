import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, tune
from mavsdk.tune import TuneDescription

from .abstract_base_plugin import AbstractBasePlugin

class Tune(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("tune", system, loop, logger)

    def play_tune(self, tune_desc : TuneDescription) -> None:
        """
        Send a tune to be played by the system

        :param tune_desc: The tune to be played
        :type tune_desc: TuneDescription        
        """
        
        super().submit_task(
            asyncio.ensure_future(self._system.tune.play_tune(tune_desc), loop=self._loop)
        )

    def play_note_c(self) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_C], 100)), loop=self._loop))
