import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import List

from mavsdk import System
from functools import partial

from .abstract_base_plugin import AbstractBasePlugin



class Ftp(AbstractBasePlugin):

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("ftp", system, loop, logger)
        self._comp_id = None

        self._comp_id_task = asyncio.ensure_future(self._system.ftp.get_our_compid(), loop=self._loop)
        self._comp_id_task.add_done_callback(partial(self._compid_callback))

    def _compid_callback(self, task: Task) -> None:
        self._comp_id = task.result()
        del self._comp_id_task

    async def _download_file(self, remote_file_path : str, local_directory : str) -> None:

        async for data in self._system.ftp.download(remote_file_path, local_directory):
            percent_downloaded : float = (data.bytes_transferred / data.total_bytes)
            self._logger.info("\rFile at remote path \"{}\" downloading to directory \"{}\": {:.2f}%      ".format(
                remote_file_path, local_directory, percent_downloaded
            ))

    async def _upload_file(self, local_file_path : str, remote_directory : str) -> None:

        async for data in self._system.ftp.upload(local_file_path, remote_directory):
            percent_uploaded : float = (data.bytes_transferred / data.total_bytes)
            self._logger.info("\rFile at local path \"{}\" uploading to directory \"{}\": {:.2f}%      ".format(
                local_file_path, remote_directory, percent_uploaded
            ))

    def get_our_component_id(self) -> int:
        """
        Get our own component ID.

        :return: Our component ID
        :rtype: uint32
        """
        return self._comp_id

    def download_file(self, remote_file_path : str, local_directory : str) -> None:
        """
        Downloads a file remotely to a local directory while logging progress

        :param remote_file_path: The path to the file to remotely download
        :type remote_file_path: str
        :param local_directory: The path to the local directory to download the file to
        :type local_directory: str
        """
        self._logger.info("Downloading the file at \"{}\" to local directory \"{}\"".format(
            remote_file_path, local_directory
        ))
        super().submit_task(
            asyncio.ensure_future(self._download_file(remote_file_path, local_directory), loop=self._loop)
        )

    def upload_file(self, local_file_path : str, remote_directory : str) -> None:
        """
        Uploads a local file to a remote directory while logging progress

        :param local_file_path: The path to the file to uploaded
        :type local_file_path: str
        :param remote_directory: The path to the remote directory to upload the file to
        :type remote_directory: str
        """
        self._logger.info("Uploading the file at \"{}\" to remote directory \"{}\"".format(
            local_file_path, remote_directory
        ))
        super().submit_task(
            asyncio.ensure_future(self._upload_file(local_file_path, remote_directory), loop=self._loop)
        )

    def create_directory(self, remote_directory_path : str) -> None:
        """
        Creates a directory remotely via FTP.

        :param remote_directory_path: The remote path of the directory to create
        :type remote_directory_path: str
        """
        self._logger.info("Creating directory at path \"{}\" via FTP".format(remote_directory_path))
        super().submit_task(
            asyncio.ensure_future(self._system.ftp.create_directory(remote_directory_path), loop=self._loop)
        )

    def remove_directory(self, remote_directory_path : str) -> None:
        """
        Removes a directory remotely via FTP.

        :param remote_directory_path: The remote path of the directory to remove
        :type remote_directory_path: str
        """
        self._logger.info("Removing directory at path \"{}\" via FTP".format(remote_directory_path))
        super().submit_task(
            asyncio.ensure_future(self._system.ftp.remove_directory(remote_directory_path), loop=self._loop)
        )

    def remove_file(self, remote_file_path : str) -> None:
        """
        Removes a file remotely via FTP.

        :param remote_file_path: The remote path of the file to remove
        :type remote_file_path: str
        """
        self._logger.info("Removing file at path \"{}\" via FTP".format(remote_file_path))
        super().submit_task(
            asyncio.ensure_future(self._system.ftp.remove_file(remote_file_path), loop=self._loop)
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
            asyncio.ensure_future(self._system.ftp.rename(remote_source_path, remote_dest_path), loop=self._loop)
        )

    def reset(self) -> None:
        """
        Resets FTP server in case there are stale open sessions.
        """
        self._logger.info("Resetting the FTP server")

        super().submit_task(
            asyncio.ensure_future(self._system.ftp.reset(), loop=self._loop)
        )

    def set_root_directory(self, root_directory : str) -> None:
        """
        Sets the root directory for MAVLink FTP server.

        :param root_directory: The path to set for the root directory of the FTP server
        :type root_directory: str
        """
        self._logger.info("Setting the root directory of the MAVLink FTP server to {}".format(root_directory))

        super().submit_task(
            asyncio.ensure_future(self._system.ftp.set_root_directory(root_directory), loop=self._loop)
        )

    def set_target_component_id(self, comp_id : int) -> None:
        """
        Sets the target's component ID. By default, it is the autopilot.

        :param comp_id: The component ID to set
        :type comp_id: uint32
        """
        self._logger.info("Setting the target's component ID to {}".format(comp_id))

        super().submit_task(
            asyncio.ensure_future(self._system.ftp.set_target_compid(comp_id), loop=self._loop)
        )

        self._comp_id = comp_id

    def are_files_identical(self, local_file_path : str, remote_file_path : str) -> bool:
        """
        Compares a local file to a remote file using a CRC32 checksum

        :param local_file_path: The path of the local file
        :type local_file_path: str
        :param remote_file_path: The path of the remote file
        :type remote_file_path: str
        :return: If the files are identical, true, otherwise false
        :rtype: bool
        """
        self._logger.info("Comparing a local file at path \"{}\" to a remote file at path \"{}\"".format(
            local_file_path, remote_file_path
        ))

        # NOTE: @Justin, is this the best way to do this since the call is not run during initialization?
        files_are_identical : bool = self._loop.run_until_complete(
            self._system.ftp.are_files_identical(local_file_path, remote_file_path)
        )

        return files_are_identical

    def list_directory(self, remote_directory : str) -> List[str]:
        """
        Lists items in the given remote directory

        :param remote_directory: The path to the remote directory to list
        :type remote_directory: str
        :return: The list of directory contents
        :rtype: List[str]
        """

        # NOTE: @Justin, is this the best way to do this since the call is not run during initialization?
        directory_contents : List[str] = self._loop.run_until_complete(
            self._system.ftp.list_directory(remote_directory)
        )

        return directory_contents

