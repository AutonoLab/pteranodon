import asyncio
from threading import Thread
from typing import List, Optional, Tuple, Any, Dict
import os
import re
import socket
from collections import defaultdict
from concurrent import futures
import logging
import sys

from pymavlink import mavutil
from serial import serialutil

try:
    import uvloop

    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
except ModuleNotFoundError:
    pass


class ServerDetector:
    """
    Searches serial devices and a server for MAV SDK server instances
    """

    def __init__(
        self,
        server_ip_addr: str = "127.0.0.1",
        port_range: Tuple[int, int] = (14540, 14550),
        serial_baud_rate: int = 115200,
        logger: Optional[logging.Logger] = None,
    ):
        """
        Searches serial devices and, if a server ip address is provided, that server for open MAV SDK servers.

        :param server_ip_addr: The server IP address to search for MAV SDK searches, None if only serial devices should be searched.
        :type server_ip_addr: Optional[str]
        """

        self._user_provided_logger = logger is not None

        self._logger: logging.Logger = (
            logger
            if self._user_provided_logger
            else ServerDetector.setup_logger("server_detect.log")
        )

        self._event_loop = asyncio.new_event_loop()

        self._server_addr = server_ip_addr
        self._serial_baud_rate = serial_baud_rate

        # Between socket kind and ports
        self._available_ports: Dict[int, List[int]] = defaultdict(list)

        self._port_range: Tuple[int, int] = port_range

        if port_range[0] < 0 or port_range[1] < 0 or port_range[0] >= port_range[1]:
            self._logger.warning(
                "Invalid port range given, reverting to default (14540, 14550)"
            )
            self._port_range = (14540, 14550)

        self._loop_thread = Thread(
            name="Detector-Loop-Thread",
            target=self._run_loop,
            daemon=True,
        )

    # setup the logger
    @staticmethod
    def setup_logger(log_file_name: str) -> logging.Logger:
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

        stdout_handler = logging.StreamHandler(sys.stdout)
        stdout_handler.setLevel(logging.DEBUG)
        stdout_handler.setFormatter(formatter)

        file_handler = logging.FileHandler(log_file_name)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)

        logger.addHandler(file_handler)
        logger.addHandler(stdout_handler)

        return logger

    @staticmethod
    def close_logger(logger: logging.Logger) -> None:
        handlers = logger.handlers[:]
        for handler in handlers:
            logger.removeHandler(handler)
            handler.close()

    def _run_loop(self):
        self._event_loop.run_forever()

    async def _test_port_open(
        self,
        port: int,
        socket_kind: int,
        timeout: float = 1.0,
    ) -> Optional[Tuple[int, bool]]:
        """
        Attempts to connect to a port with a specified timeout and socket kind to determine if the port is open.

        :param port: The port to check
        :type port: int
        :param socket_kind: The type of socket to open (SOCK_STREAM for TCP, SOCK_DGRAM for UDP)
        :type socket_kind: socket.SocketKind
        :param timeout: The timeout for the connect call
        :type timeout: float
        :return: The port tested and whether the port is open, None if invalid parameters
        :rtype: Optional[Tuple[int, bool]]
        """

        # Only supporting TCP and UDP
        if socket_kind not in [socket.SOCK_STREAM, socket.SOCK_DGRAM]:
            return None

        with socket.socket(socket.AF_INET, socket_kind) as sock:
            sock.settimeout(timeout)
            try:
                sock.connect((self._server_addr, port))
            except TimeoutError:
                return port, False  # Timed out
            except ConnectionError:
                return port, False  # Not open

        return port, True

    def _fetch_possible_serial_devs(self) -> List[str]:
        """
        Fetches all the possible serial devices from /sys/class/tty while removing standard devices and virtual consoles.

        :return: The paths of connected serial devices
        :rtype: List[str]
        """

        if not os.path.exists("/sys/class/tty"):
            self._logger.warning(
                '"/sys/class/tty" does not exist on this file system. Could not check for serial devices'
            )
            return []

        blacklisted_devices = ["pmtx", "console", "ttyprintk"]

        serial_devices = os.listdir("/sys/class/tty")

        # Remove blacklisted files
        serial_devices = [
            dev for dev in serial_devices if dev not in blacklisted_devices
        ]

        # Remove virtual consoles (matches "tty" followed by any number and nothing else)
        serial_devices = [
            dev for dev in serial_devices if re.match("(tty)[0-9]*$", dev) is None
        ]

        serial_paths = ["/dev/" + dev for dev in serial_devices]

        return serial_paths

    async def fetch_open_proto_ports(
        self, socket_kind: int, fetch_cached: bool = False
    ) -> List[int]:
        """
        Fetches the open ports for the given protocol (not necessarily open for MAVSDK). Blocking.

        :param socket_kind: The kind of socket to test (SOCK_STREAM for TCP, SOCK_DGRAM for UDP)
        :type socket_kind: socket.SocketKind
        :param fetch_cached: Whether to search for open ports or use cached data
        :type fetch_cached: bool
        :return: A list of open UDP or TCP ports
        :rtype: List[int]
        """
        # Only supporting TCP and UDP
        if socket_kind not in [socket.SOCK_STREAM, socket.SOCK_DGRAM]:
            return []

        if fetch_cached:
            return self._available_ports[socket_kind]

        all_ports = range(self._port_range[0], self._port_range[1] + 1)

        port_data = await asyncio.gather(
            *[self._test_port_open(port, socket_kind) for port in all_ports],
        )

        port_data = [
            port_tup for port_tup in port_data if port_tup is not None
        ]  # Remove None tuples
        open_ports = [
            port_tup[0] for port_tup in port_data if port_tup[1]
        ]  # Filter only open ports

        self._available_ports[socket_kind] = open_ports
        return open_ports

    async def _test_server(
        self,
        serial_dev: Optional[str] = None,
        port: Optional[int] = None,
        is_tcp: Optional[bool] = None,
        conn_timeout: float = 1.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Tests the given server (or serial device) and returns the input arguments if a MAV SDK server is present.

        :param serial_dev: The path to the serial device to test (None if not testing serial)
        :type serial_dev: Optional[str]
        :param port: The UDP or TCP port to test (None if not testing TCP or UDP)
        :type port: Optional[int]
        :param is_tcp: True if testing TCP, False if testing UDP, None if testing serial
        :type is_tcp: Optional[bool]
        :param conn_timeout: Timeout for heartbeat message wait
        :type conn_timeout: float
        :return: None if no server at path, a dictionary of parameters otherwise
        :rtype: Optional[Dict[str, Any]]
        """
        if serial_dev is None and port is None and is_tcp is None:
            return None

        conn: Optional[mavutil.mavfile] = None

        if serial_dev is not None:
            try:
                conn = mavutil.mavlink_connection(device=serial_dev, baud=115200)
            except serialutil.SerialException:
                return None

        if port is not None:
            prefix = "tcp" if is_tcp else "udp"
            addr = f"{prefix}:{self._server_addr}:{port}"
            conn = mavutil.mavlink_connection(addr)

        # Invalid parameters
        if conn is None:
            return None

        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=conn_timeout)

        conn.close()

        # Msg is None on timeout
        if msg is None:
            return None

        return {
            "serial_dev": serial_dev,
            "port": port,
            "is_tcp": is_tcp,
        }

    # For the given dictionary, get the corresponding string server path
    def _get_server_value(self, data_dict: Optional[Dict[str, Any]]) -> Optional[str]:
        if data_dict is None:
            return None

        serial_dev: Optional[str] = data_dict["serial_dev"]
        port: Optional[int] = data_dict["port"]

        if serial_dev is not None:
            return f"serial://{serial_dev}:{self._serial_baud_rate}"

        addr = self._server_addr if self._server_addr != "127.0.0.1" else ""

        if data_dict["is_tcp"]:
            return f"tcp://{addr}:{port}"

        return f"udp://{addr}:{port}"

    async def _get_mavsdk_servers_data(
        self, test_cached: bool
    ) -> List[List[Optional[Dict]]]:
        serial_devs = self._fetch_possible_serial_devs()

        tcp_ports = await self.fetch_open_proto_ports(
            socket.SOCK_STREAM, fetch_cached=test_cached
        )
        udp_ports = await self.fetch_open_proto_ports(
            socket.SOCK_DGRAM, fetch_cached=test_cached
        )

        serial_devs_future = asyncio.gather(
            *[self._test_server(serial_dev=dev) for dev in serial_devs]
        )

        tcp_ports_future = asyncio.gather(
            *[self._test_server(port=port, is_tcp=True) for port in tcp_ports]
        )

        udp_ports_future = asyncio.gather(
            *[self._test_server(port=port, is_tcp=False) for port in udp_ports]
        )

        all_data = await asyncio.gather(
            serial_devs_future, tcp_ports_future, udp_ports_future
        )

        # This is the correct type. `gather` states that it returns lists,
        #   it returns lists in the code, but mypy believes it returns a tuple.
        return [*all_data]

    def get_mavsdk_servers(
        self, test_cached: bool = False, timeout: float = 10.0
    ) -> List[str]:
        """
        Fetch the list of serial devices, complete TCP paths, or complete UDP paths
        that have MAVSDK servers running on them. Blocking.

        :return: The list of server paths
        :rtype: List[str]
        """

        self._loop_thread.start()

        new_future: futures.Future = asyncio.run_coroutine_threadsafe(
            self._get_mavsdk_servers_data(test_cached), loop=self._event_loop
        )

        try:
            all_data: List[List[Optional[Dict[str, Any]]]] = new_future.result(timeout)
        except futures.TimeoutError:
            self._logger.error(
                "Could not fetch servers before timeout. Try again with a larger timeout or a smaller port range."
            )
            raise TimeoutError

        full_servers_list: List[str] = []

        for data_list in all_data:
            servers_opt_list = [self._get_server_value(data) for data in data_list]
            servers_list: List[str] = [
                server for server in servers_opt_list if server is not None
            ]  # Filter invalid servers
            full_servers_list += servers_list

        self._event_loop.call_soon_threadsafe(self._event_loop.stop)
        self._loop_thread.join()

        if not self._user_provided_logger:
            ServerDetector.close_logger(self._logger)

        return full_servers_list
