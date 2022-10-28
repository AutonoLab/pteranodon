import asyncio
from typing import List, Optional, Tuple, Any, Dict
import os
import re
import socket

from pymavlink import mavutil


# TODO_: Better name?
class AutoConnect:
    """
    Searches serial devices and, if a server ip address is provided, that server for open MAV SDK servers.
    """

    def __init__(self, server_ip_addr: Optional[str]):
        """
        Searches serial devices and, if a server ip address is provided, that server for open MAV SDK servers.

        :param server_ip_addr: The server IP address to search for MAV SDK searches, None if only serial devices should be searched.
        :type server_ip_addr: Optional[str]
        """
        self._server_addr = server_ip_addr
        self._should_check_server = self._server_addr is not None

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
        # Must have a server address
        if not self._should_check_server:
            return None

        # Only supporting TCP and UDP
        if socket_kind not in [socket.SOCK_STREAM, socket.SOCK_DGRAM]:
            return None

        with socket.socket(socket.AF_INET, socket_kind) as sock:
            sock.settimeout(timeout)
            try:
                sock.connect((self._server_addr, port))
            except TimeoutError:
                return port, False

        return port, True

    def fetch_possible_serial_devs(self) -> List[str]:
        """
        Fetches all the possible serial devices from /sys/class/tty while removing standard devices and virtual consoles.

        :return: The paths of connected serial devices
        :rtype: List[str]
        """
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

    # Must be done in parallel, would take 17+ hours otherwise
    def fetch_open_proto_ports(self, socket_kind: int) -> List[int]:
        """
        Fetches the open ports for the given protocol (not necessarily open for MAVSDK). Blocking.

        :param socket_kind: The kind of socket to test (SOCK_STREAM for TCP, SOCK_DGRAM for UDP)
        :type socket_kind: socket.SocketKind
        :return: A list of open UDP or TCP ports
        :rtype: List[int]
        """
        # Only supporting TCP and UDP
        if socket_kind not in [socket.SOCK_STREAM, socket.SOCK_DGRAM]:
            return []

        port_range = range(1024, pow(2, 16))

        new_loop = asyncio.new_event_loop()
        ports_future = asyncio.gather(
            *[self._test_port_open(port, socket_kind) for port in port_range]
        )
        port_data = new_loop.run_until_complete(ports_future)

        port_data = [
            port_tup for port_tup in port_data if port_tup is not None
        ]  # Remove None tuples
        open_ports = [
            port_tup[0] for port_tup in port_data if port_tup[1]
        ]  # Filter only open ports
        new_loop.stop()
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
            conn = mavutil.mavlink_connection(device=serial_dev, baud=115200)

        if port is not None and self._should_check_server:
            prefix = "tcp" if is_tcp else "udp"
            addr = f"{prefix}:{self._server_addr}:{port}"
            conn = mavutil.mavlink_connection(addr)

        # Invalid parameters
        if conn is None:
            return None

        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=conn_timeout)

        # Msg is None on timeout
        if msg is None:
            return None

        return {
            "serial_dev": serial_dev,
            "port": port,
            "is_tcp": is_tcp,
        }

    def get_mavsdk_servers(self) -> List[str]:
        """
        Fetch the list of serial devices, complete TCP paths, or complete UDP paths
        that have MAVSDK servers running on them. Blocking.

        :return: The list of server paths
        :rtype: List[str]
        """
        serial_devs = self.fetch_possible_serial_devs()

        serial_devs_future = asyncio.gather(
            *[self._test_server(serial_dev=dev) for dev in serial_devs]
        )

        all_data_future = serial_devs_future

        if self._should_check_server:
            tcp_ports = self.fetch_open_proto_ports(socket.SOCK_STREAM)
            udp_ports = self.fetch_open_proto_ports(socket.SOCK_DGRAM)

            tcp_ports_future = asyncio.gather(
                *[self._test_server(port=port, is_tcp=True) for port in tcp_ports]
            )
            udp_ports_future = asyncio.gather(
                *[self._test_server(port=port, is_tcp=False) for port in udp_ports]
            )

            all_data_future = asyncio.gather(
                serial_devs_future, tcp_ports_future, udp_ports_future
            )

        new_loop = asyncio.new_event_loop()
        all_data = new_loop.run_until_complete(all_data_future)

        # For the given dictionary, get the corresponding string server path
        def get_server_value(data_dict: Optional[Dict[str, Any]]) -> Optional[str]:
            if data_dict is None:
                return None

            serial_dev: Optional[str] = data_dict["serial_dev"]
            port: Optional[int] = data_dict["port"]

            if serial_dev is not None:
                return serial_dev

            if not self._should_check_server:
                return None

            if data_dict["is_tcp"]:
                return f"tcp:{self._server_addr}:{port}"

            return f"udp:{self._server_addr}:{port}"

        # There are only serial devs, represent as a list for for-loop
        if not self._should_check_server:
            all_data = [all_data]

        full_servers_list: List[str] = []

        for data_list in all_data:
            servers_list = [get_server_value(data) for data in data_list]
            servers_list = [
                server for server in servers_list if server is not None
            ]  # Filter invalid servers
            full_servers_list += servers_list

        new_loop.stop()
        return full_servers_list
