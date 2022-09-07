import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List

from mavsdk import System, ftp

from .abstract_base_plugin import AbstractBasePlugin



class Ftp(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("ftp", system, loop, logger)
        self._comp_id = None

        self._comp_id_task = asyncio.ensure_future(self._get_our_compid(), loop=self._loop)

    async def _get_our_compid(self) -> None:
        while True:
            self._id = await self._system.ftp.get_our_compid()
            break

    def get_our_component_id(self) -> int:
        """
        Get our own component ID.

        :return: Our component ID
        :rtype: uint32
        """
        return self._comp_id

    def create_directory(self, remote_directory_path : str) -> None:
        """
        Creates a directory remotely via FTP.

        :param remote_directory_path: The remote path of the directory to create
        :type remote_directory_path: str
        """
        self._logger.info("Creating directory at path \"{}\" via FTP".format(remote_directory_path))
        super().submit_task(
            asyncio.ensure_future(self._system.ftp.create_directory(remote_directory_path))
        )

    def remove_directory(self, remote_directory_path : str) -> None:
        """
        Removes a directory remotely via FTP.

        :param remote_directory_path: The remote path of the directory to remove
        :type remote_directory_path: str
        """
        self._logger.info("Removing directory at path \"{}\" via FTP".format(remote_directory_path))
        super().submit_task(
            asyncio.ensure_future(self._system.ftp.remove_directory(remote_directory_path))
        )

    def remove_file(self, remote_file_path : str) -> None:
        """
        Removes a file remotely via FTP.

        :param remote_file_path: The remote path of the file to remove
        :type remote_file_path: str
        """
        self._logger.info("Removing file at path \"{}\" via FTP".format(remote_file_path))
        super().submit_task(
            asyncio.ensure_future(self._system.ftp.remove_file(remote_file_path))
        )

    def rename(self, remote_source_path : str, remote_dest_path : str) -> None:
        """
        Renames (moves) a remote file or directory from the given source path to the destination path.

        :param remote_source_path: The path of the file to be renamed (moved)
        :type remote_source_path: str
        :param remote_dest_path: The path of the location to rename (move) the file to
        :type remote_dest_path: str
        """
        self._logger.info("Moving a remote file/directory from \"{}\" to \"{}\" via FTP".format(remote_source_path,
                                                                                                remote_dest_path))
        super().submit_task(
            asyncio.ensure_future(self._system.ftp.rename(remote_source_path, remote_dest_path))
        )

    def reset(self) -> None:
        """
        Resets FTP server in case there are stale open sessions.
        """
        self._logger.info("Resetting the FTP server")

        super().submit_task(
            asyncio.ensure_future(self._system.ftp.reset())
        )

    def set_root_directory(self, root_directory : str) -> None:
        """
        Sets the root directory for MAVLink FTP server.

        :param root_directory: The path to set for the root directory of the FTP server
        :type root_directory: str
        """
        self._logger.info("Setting the root directory of the MAVLink FTP server to {}".format(root_directory))

        super().submit_task(
            asyncio.ensure_future(self._system.ftp.set_root_directory(root_directory))
        )
