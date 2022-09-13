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

    def play_note_c(self, note : str) -> None:
        """
        Send a single note to be played by the system

        :param note: The letter of the note to be played
        :type note: str
        """

        if(note.upper() == 'C'):
            super().submit_task(
                asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_C], 100)), loop=self._loop))
                
        elif(note.upper() == 'D'):
            super().submit_task(
                asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_D], 100)), loop=self._loop))
                
        elif(note.upper() == 'E'):
            super().submit_task(
                asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_E], 100)), loop=self._loop))
                
        elif(note.upper() == 'F'):
            super().submit_task(
                asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_F], 100)), loop=self._loop))
                
        elif(note.upper() == 'G'):
            super().submit_task(
                asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_G], 100)), loop=self._loop))
                
        elif(note.upper() == 'A'):
            super().submit_task(
                asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_A], 100)), loop=self._loop))
                
        elif(note.upper() == 'B'):
            super().submit_task(
                asyncio.ensure_future(self._system.tune.play_tune(TuneDescription([tune.STYLE_LEGATTO, tune.DURATION_2, tune.NOTE_B], 100)), loop=self._loop))
            