import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import List

from mavsdk import System
from mavsdk.log_files import Entry
from mavsdk.log_files import ProgressData
from functools import partial
from threading import Condition

from .abstract_base_plugin import AbstractBasePlugin


class LogFiles(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("LogFiles", system, loop, logger)

        self._download_progress = None
        self._entry_list = None
        self._entry_list_task = asyncio.ensure_future(self._system.log_files.get_entries(), loop=self._loop)
        self._entry_list_task.add_done_callback(partial(self._get_entries_callback))

    def download_log_file(self, entry : Entry, path : str) -> ProgressData:
        """
        Download log file synchronously.

        :param entry: Entry of the log file to download.
        :type entry: Entry        
        :param path: Path of where to download log file to.
        :type path: str
        :return: Download progress
        :rtype: ProgressData
        """

        download_progress_task = asyncio.ensure_future(self._system.log_files.download_log_file(entry, path), loop=self._loop)

        done_condition = Condition()

        # When task is done, stop waiting
        download_progress_task.add_done_callback(lambda _: done_condition.notify())

        # Wait with a timeout of 1 second
        done_condition.wait(1.0)

        self._logger.info("Downloading log file with id: {}".format(entry.id))

        try:
            return download_progress_task.result()
        except asyncio.InvalidStateError:
            # If the result is not available yet,
            #       it can be assumed that the wait call timed out before the callback was done
            self._logger.error("Could not return log file download progress! Request timed out!")
            return None


        super().submit_task(
            asyncio.ensure_future(self._system.log_files.download_log_file(entry, path), loop=self._loop)
        )

    def get_download_progess(self) -> ProgressData:
        """
        Get the progress of a download

        :return: Download progress
        :rtype: ProgressData
        """
        return self._download_progress_task

    def erase_all_log_files(self) -> None:
        """
        Erase all log files.
        """

        self._logger.info("Erased all log files!")

        super().submit_task(
            asyncio.ensure_future(self._system.log_files.erase_all_log_files(), loop=self._loop)
        )

    def _get_entries_callback(self, task : Task) -> None:
        self._entry_list = task.result()
        del self._entry_list_task

    def get_entries(self) -> List[Entry]:
        """
        Get List of log files.

        :return: Log entry list
        :rtype: List[Entry]
        """

        return self._entry_list
