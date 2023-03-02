import asyncio
import logging
from . import connection, msg

from . import DEBUG_LEVEL
logger = logging.getLogger(__name__)
logger.setLevel(DEBUG_LEVEL)


class Subscriber(object):
    """Receives data from the Gazebo publish-subscribe bus.

    :ivar _topic: (str) The topic name this subscriber is listening for.
    :ivar _msg_type: (str) The Gazebo message type.
    :ivar _callback: (function) The current function to invoke.
    """
    def __init__(self, topic: str, msg_type: str, callback, local_host, local_port):
        """:class:`Subscriber` should not be directly created"""
        logger.debug(f'Subscriber.__init__({local_host}, {local_port}')
        self._topic = topic
        self._msg_type = msg_type
        self._callback = callback
        self._stop_connection = False

        self._local_host = local_host
        self._local_port = local_port
        self._connection_future = asyncio.Future()
        self._connections = []

    async def remove(self):
        """Stop listening for this topic.

        Note: Once :func:`remove` is called, the callback will no
        longer be invoked.
        """
        self._stop_connection = True
        for conn in self._connections:
            await conn.close()

    async def _deallocate_connection(self, _connection):
        self._connections.remove(_connection)
        await _connection.close()
        if len(self._connections) == 0:
            self._connection_future = asyncio.Future()

    def wait_for_connection(self):
        return self._connection_future

    def subscribe(self, pub):
        asyncio.ensure_future(self._connect(pub))

    async def _connect(self, pub):
        try:
            _connection = connection.Connection(f'subscriber_{self._topic}')

            # Connect to the remote provider.
            await _connection.connect(pub.host, pub.port)
            self._connections.append(_connection)

            # Send the initial message, which is encapsulated inside of a
            # Packet structure.
            to_send = msg.subscribe_pb2.Subscribe()
            to_send.topic = pub.topic
            to_send.host = self._local_host
            to_send.port = self._local_port
            to_send.msg_type = pub.msg_type
            to_send.latching = False

            await _connection.write_packet('sub', to_send, timeout=60)
            self._connection_future.set_result(None)

            while not self._stop_connection:
                data = await _connection.read_raw()
                if data is None:
                    await self._deallocate_connection(_connection)
                    return

                self._callback(data)
        except Exception as e:
            logger.exception(f"EXCEPTION HANDLING THE SUBSCRIPTION: {e}")
            raise e
