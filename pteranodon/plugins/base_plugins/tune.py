from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from mavsdk.tune import TuneDescription, SongElement

from .abstract_base_plugin import AbstractBasePlugin


class Tune(AbstractBasePlugin):
    """
    Enable creating and sending a tune to be played on the system.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("tune", system, loop, logger)

    def play_tune(self, tune_desc: TuneDescription) -> None:
        """
        Send a tune to be played by the system

        :param tune_desc: The tune to be played
        :type tune_desc: TuneDescription
        """

        self._submit_coroutine(self._system.tune.play_tune(tune_desc))

    def play_note(self, note: str) -> None:
        """
        Send a single note to be played by the system

        :param note: The letter of the note to be played
        :type note: str
        """

        tune_description = TuneDescription(
            [SongElement.STYLE_LEGATO, SongElement.DURATION_2], 100
        )

        if note.upper() == "C":
            tune_description.song_elements.append(SongElement.NOTE_C)
        elif note.upper() == "D":
            tune_description.song_elements.append(SongElement.NOTE_D)
        elif note.upper() == "E":
            tune_description.song_elements.append(SongElement.NOTE_E)
        elif note.upper() == "F":
            tune_description.song_elements.append(SongElement.NOTE_F)
        elif note.upper() == "G":
            tune_description.song_elements.append(SongElement.NOTE_G)
        elif note.upper() == "A":
            tune_description.song_elements.append(SongElement.NOTE_A)
        elif note.upper() == "B":
            tune_description.song_elements.append(SongElement.NOTE_B)
        else:
            raise Exception("Unkown note type")

        self._submit_coroutine(self._system.tune.play_tune(tune_description))
