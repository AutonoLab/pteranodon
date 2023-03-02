from concurrent import futures
import time
import math
import sys
import asyncio
import logging
from . import msg
from .parse_error import ParseError

from . import DEBUG_LEVEL
logger = logging.getLogger(__name__)
logger.setLevel(DEBUG_LEVEL)


async def _wait_closed(stream):
    assert(sys.version_info.major >= 3)
    if sys.version_info.minor >= 7:
        await stream.wait_closed()


class DisconnectError(Exception):
    def __init__(self,
                 connection_name: str,
                 server_addr: tuple,
                 local_addr: tuple,
                 discarded_bytes: int):
        """
        :param connection_name: Name of the connection
        :param server_addr: remote address of the connection (address, port)
        :type server_addr: tuple[str, int]
        :param local_addr: local address of the connection (address, port)
        :type local_addr: tuple[str, int]
        :param discarded_bytes: number of bytes not read from the socket
        """
        self._connection_name = connection_name
        self._server_addr = server_addr
        self._local_addr = local_addr
        self._discarded_bytes = discarded_bytes

    @staticmethod
    def _to_addr(addr):
        return f'{addr[0]}:{addr[1]}'

    def __str__(self):
        return f'DisconnectError' \
            f'({self._connection_name}: {self._to_addr(self._local_addr)} -> {self._to_addr(self._server_addr)})' + \
            (f' bytes not collected: {self._discarded_bytes}' if self._discarded_bytes is not None and self._discarded_bytes > 0 else '')


class Server(object):
    def __init__(self, name: str):
        self._name = name
        self._server = None
        self._listen_host = None
        self._listen_port = None
        self._running_server = None

    async def serve(self, handler):
        """
        Start TCP server
        :param handler: called for each new connection. async function
        :type handler: async lambda reader, writer -> None
        :return:
        """
        self._server = await asyncio.start_server(handler, host='0.0.0.0')

        self._listen_host, self._listen_port = self._server.sockets[0].getsockname()
        logger.info(f"Listening on {self._listen_port}:{self._listen_port}")

        self._running_server = asyncio.ensure_future(self._server_loop())

        return self._listen_host, self._listen_port

    async def _server_loop(self):
        if sys.version_info.minor >= 7:
            async with self._server:
                await self._server.serve_forever()
        else:
            await self._server.wait_closed()

    async def close(self):
        self._server.close()
        await _wait_closed(self._server)
        try:
            await asyncio.wait_for(self._running_server, timeout=2)
        except asyncio.CancelledError:
            print('futures.CancelledError')
        except asyncio.TimeoutError:
            print('asyncio.TimeoutError')

    @property
    def listen_host(self):
        assert self._server is not None
        return self._listen_host

    @property
    def listen_port(self):
        assert self._server is not None
        return self._listen_port


class Connection(object):
    """Manages a Gazebo protocol connection.
    """

    def __init__(self, name):
        self.name = name
        self._address = None
        self._port = None
        self._reader = None
        self._writer = None
        self._closed = True

    async def connect(self, address, port):
        logger.debug('Connection.connect')
        self._address = address
        self._port = port

        reader, writer = await asyncio.open_connection(address, port)
        self.accept_connection(reader, writer)

    def accept_connection(self, reader, writer):
        self._reader = reader
        self._writer = writer
        self._closed = False

    async def close(self):
        if self._closed:
            logger.debug("Trying to close an already closed connection")
            return
        self._closed = True

        self._writer.write_eof()
        await self._writer.drain()
        self._writer.close()
        await _wait_closed(self._writer)

    async def write_packet(self, name: str, message, timeout):
        assert not self._closed
        packet = msg.packet_pb2.Packet()
        cur_time = time.time()
        packet.stamp.sec = int(cur_time)
        packet.stamp.nsec = int(math.fmod(cur_time, 1) * 1e9)
        packet.type = name.encode()
        packet.serialized_data = message.SerializeToString()
        await self._write(packet.SerializeToString(), timeout)

    async def write(self, message, timeout=None):
        data = message.SerializeToString()
        await self._write(data, timeout)

    async def _write(self, data, timeout):
        header = ('%08X' % len(data)).encode()
        self._writer.write(header + data)
        await asyncio.wait_for(self._writer.drain(), timeout=timeout)

    async def read_raw(self):
        """
        Read incoming packet without parsing it
        :return: byte array of the packet
        """
        header = None
        try:
            assert not self._closed
            header = await self._reader.readexactly(8)
            if len(header) < 8:
                raise ParseError('malformed header: ' + str(header))

            try:
                size = int(header, 16)
            except ValueError:
                raise ParseError('invalid header: ' + str(header))
            else:
                data = await self._reader.readexactly(size)
                return data
        except asyncio.exceptions.IncompleteReadError:
            return None
        except (ConnectionResetError, asyncio.IncompleteReadError) as e:
            if self._closed:
                return None
            else:
                local_addr, local_port = self._writer.transport.get_extra_info('sockname')
                discarded_bytes = len(e.partial) if isinstance(e, asyncio.IncompleteReadError) else None
                if header is not None:
                    discarded_bytes += 8
                raise DisconnectError(
                    connection_name=self.name,
                    server_addr=(self._address, self._port),
                    local_addr=(local_port, local_addr),
                    discarded_bytes=discarded_bytes
                ) from e

    async def read_packet(self):
        data = await self.read_raw()
        if not self._closed:
            packet = msg.packet_pb2.Packet.FromString(data)
            return packet
