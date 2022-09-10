import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from mavsdk.log_files import Entry
from mavsdk.log_files import Entry
from mavsdk.log_files import ProgressData

from .abstract_base_plugin import AbstractBasePlugin


class LogFiles(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("LogFiles", system, loop, logger)


    def download_log_file(self, entry : Entry, path : str) -> ProgressData:
        """
        Download log file synchronously.

        :param entry: Entry of the log file to download.
        :type entry: Entry
        
        :param path: Path of where to download log file to.
        :type path: str
        """

        super().submit_task(
            asyncio.ensure_future(self._system.log_files.download_log_file(entry, path), loop=self._loop)
        )

    def erase_all_log_files(self) -> None:
        """
        Erase all log files.
        """

        super().submit_task(
            asyncio.ensure_future(self._system.log_files.erase_all_log_files(), loop=self._loop)
        )

    def get_entries(self) -> Entry:
        """
        Get List of log files.
        """

        super().submit_task(
            asyncio.ensure_future(self._system.log_files.get_entries(), loop=self._loop)
        )
