import typing
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List

from mavsdk import System


from .abstract_base_plugin import AbstractBasePlugin


class Ftp(AbstractBasePlugin):
    """
    Implements file transfer functionality using MAVLink FTP.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("ftp", system, loop, logger)
        self._comp_id: typing.Optional[int] = None
        self._root_directory = "/"

        self._comp_id = self._loop.run_until_complete(self._system.ftp.get_our_compid())

        self._end_init()

    async def _download_file(self, remote_file_path: str, local_directory: str) -> None:
        async for data in self._system.ftp.download(remote_file_path, local_directory):
            percent_downloaded: float = data.bytes_transferred / data.total_bytes
            self._logger.info(
                (
                    f'\rFile at remote path "{remote_file_path}"',
                    f' downloading to directory "{local_directory}": {percent_downloaded:.2f}%      ',
                )
            )
            if percent_downloaded == 1.0:
                break

    async def _upload_file(self, local_file_path: str, remote_directory: str) -> None:
        async for data in self._system.ftp.upload(local_file_path, remote_directory):
            percent_uploaded: float = data.bytes_transferred / data.total_bytes
            self._logger.info(
                (
                    f'\rFile at local path "{local_file_path}"',
                    f' uploading to directory "{remote_directory}": {percent_uploaded:.2f}%      ',
                )
            )
            if percent_uploaded == 1.0:
                break

    def get_our_component_id(self) -> typing.Optional[int]:
        """
        Get our own component ID.

        :return: Our component ID
        :rtype: uint32
        """
        return self._comp_id

    def download(self, remote_file_path: str, local_directory: str) -> None:
        """
        Downloads a file remotely to a local directory while logging progress

        :param remote_file_path: The path to the file to remotely download
        :type remote_file_path: str
        :param local_directory: The path to the local directory to download the file to
        :type local_directory: str
        """
        self._logger.info(
            f'Downloading the file at "{remote_file_path}" to local directory "{local_directory}"'
        )
        self._submit_coroutine(self._download_file(remote_file_path, local_directory))

    def upload(self, local_file_path: str, remote_directory: str) -> None:
        """
        Uploads a local file to a remote directory while logging progress

        :param local_file_path: The path to the file to uploaded
        :type local_file_path: str
        :param remote_directory: The path to the remote directory to upload the file to
        :type remote_directory: str
        """
        self._logger.info(
            f'Uploading the file at "{local_file_path}" to remote directory "{remote_directory}"'
        )
        self._submit_coroutine(self._upload_file(local_file_path, remote_directory))

    def create_directory(self, remote_directory_path: str) -> None:
        """
        Creates a directory remotely via FTP.

        :param remote_directory_path: The remote path of the directory to create
        :type remote_directory_path: str
        """
        self._logger.info(
            f'Creating directory at path "{remote_directory_path}" via FTP'
        )
        self._submit_coroutine(self._system.ftp.create_directory(remote_directory_path))

    def remove_directory(self, remote_directory_path: str) -> None:
        """
        Removes a directory remotely via FTP.

        :param remote_directory_path: The remote path of the directory to remove
        :type remote_directory_path: str
        """
        self._logger.info(
            f'Removing directory at path "{remote_directory_path}" via FTP'
        )
        self._submit_coroutine(self._system.ftp.remove_directory(remote_directory_path))

    def remove_file(self, remote_file_path: str) -> None:
        """
        Removes a file remotely via FTP.

        :param remote_file_path: The remote path of the file to remove
        :type remote_file_path: str
        """
        self._logger.info(f'Removing file at path "{remote_file_path}" via FTP')
        self._submit_coroutine(self._system.ftp.remove_file(remote_file_path))

    def rename(self, remote_source_path: str, remote_dest_path: str) -> None:
        """
        Renames (moves) a remote file or directory from the given source path to the destination path.

        :param remote_source_path: The path of the file to be renamed (moved)
        :type remote_source_path: str
        :param remote_dest_path: The path of the location to rename (move) the file to
        :type remote_dest_path: str
        """
        self._logger.info(
            f'Moving a remote file/directory from "{remote_source_path}" to "{remote_dest_path}" via FTP'
        )
        self._submit_coroutine(
            self._system.ftp.rename(remote_source_path, remote_dest_path)
        )

    def reset(self) -> None:
        """
        Resets FTP server in case there are stale open sessions.
        """
        self._logger.info("Resetting the FTP server")

        self._submit_coroutine(self._system.ftp.reset())

    def set_root_directory(self, root_directory: str) -> None:
        """
        Sets the root directory for MAVLink FTP server.

        :param root_directory: The path to set for the root directory of the FTP server
        :type root_directory: str
        """
        self._logger.info(
            f"Setting the root directory of the MAVLink FTP server to {root_directory}"
        )

        self._submit_coroutine(self._system.ftp.set_root_directory(root_directory))

        self._root_directory = root_directory

    def set_target_compid(self, comp_id: int) -> None:
        """
        Sets the target's component ID. By default, it is the autopilot.

        :param comp_id: The component ID to set
        :type comp_id: uint32
        """
        self._logger.info(f"Setting the target's component ID to {comp_id}")

        self._submit_coroutine(self._system.ftp.set_target_compid(comp_id))

        self._comp_id = comp_id

    def are_files_identical(
        self, local_file_path: str, remote_file_path: str
    ) -> typing.Optional[bool]:
        """
        Compares a local file to a remote file using a CRC32 checksum

        :param local_file_path: The path of the local file
        :type local_file_path: str
        :param remote_file_path: The path of the remote file
        :type remote_file_path: str
        :return: If the files are identical, true, otherwise false. If the request times out, returns None.
        :rtype: Optional[bool]
        """

        files_are_identical = self._submit_blocking_coroutine(
            self._system.ftp.are_files_identical(local_file_path, remote_file_path)
        )

        if files_are_identical is None:
            self._logger.error("Could not return are_files_identical result!")

        return files_are_identical

    def list_directory(self, remote_directory: str) -> List[str]:
        """
        Lists items in the given remote directory

        :param remote_directory: The path to the remote directory to list
        :type remote_directory: str
        :return: The list of directory contents
        :rtype: List[str]
        """

        files_list_opt = self._submit_blocking_coroutine(
            self._system.ftp.list_directory(remote_directory)
        )
        if files_list_opt is None:
            self._logger.error(
                "Could not return list of directory contents! Request timed out!"
            )
            return []

        return files_list_opt

    @property
    def root_directory(self) -> str:
        """
        Root directory of MAVLink FTP Server
        :return: root directory
        :rtype: str
        """
        return self._root_directory
