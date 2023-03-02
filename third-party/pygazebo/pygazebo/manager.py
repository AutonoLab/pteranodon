import asyncio
import logging
from . import connection
from .parse_error import ParseError
from .publisher import Publisher, PublisherRecord
from .subscriber import Subscriber
from . import msg

from . import DEBUG_LEVEL
logger = logging.getLogger(__name__)
logger.setLevel(DEBUG_LEVEL)


class Manager(object):
    def __init__(self, address: str, port: int):
        self._address = address
        self._port = port
        self._master = connection.Connection("Manager_master")
        self._server = connection.Server("Manager_server")
        self._clients = []
        self._namespaces = []
        self._publisher_records = set()
        self._publishers = {}
        self._subscribers = {}
        self._stop = True

    async def start(self):
        await self._run()

    async def stop(self):
        self._stop = True
        for conn in self._clients:
            await conn.close()
        await self._master.close()
        await self._server.close()

    async def advertise(self, topic_name: str, msg_type: str):
        """Inform the Gazebo server of a topic we will publish.

        :param topic_name: the topic to send data on
        :type topic_name: string
        :param msg_type: the Gazebo message type string
        :type msg_type: string
        :rtype: :class:`Publisher`
        """
        if topic_name in self._publishers:
            raise RuntimeError('multiple publishers for: ' + topic_name)

        to_send = msg.publish_pb2.Publish()
        to_send.topic = topic_name
        to_send.msg_type = msg_type
        to_send.host = self._server.listen_host
        to_send.port = self._server.listen_port

        publisher = Publisher(topic=topic_name, msg_type=msg_type)
        self._publishers[topic_name] = publisher

        await self._master.write_packet('advertise', to_send, timeout=60)
        return publisher

    async def subscribe(self, topic_name: str, msg_type: str, callback):
        """Request the Gazebo server send messages on a specific topic.

        :param topic_name: the topic for which data will be sent
        :param msg_type: the Gazebo message type string
        :param callback: A callback to invoke when new data on
              this topic is received.  The callback will be invoked
              with raw binary data.  It is expected to deserialize the
              message using the appropriate protobuf definition.
        :rtype: :class:`Subscriber`
        """

        if topic_name in self._subscribers:
            raise RuntimeError('multiple subscribers for: ' + topic_name)

        to_send = msg.subscribe_pb2.Subscribe()
        to_send.topic = topic_name
        to_send.msg_type = msg_type
        to_send.host = self._server.listen_host
        to_send.port = self._server.listen_port
        to_send.latching = False

        subscriber = Subscriber(topic=topic_name,
                                msg_type=msg_type,
                                callback=callback,
                                local_host=to_send.host,
                                local_port=to_send.port)
        self._subscribers[topic_name] = subscriber
        await self._master.write_packet('subscribe', to_send, timeout=60)
        return subscriber

    def publications(self):
        """Enumerate the current list of publications.

        :returns: the currently known publications
        :rtype: list of (topic_name, msg_type)
        """
        return [(x.topic, x.msg_type) for x in self._publisher_records]

    def namespaces(self):
        """Enumerate the currently known namespaces.

        :returns: the currently known namespaces
        :rtype: list of strings
        """
        return self._namespaces

    async def _run(self):
        """Starts the connection and processes events."""
        logger.debug('Manager.run')
        master_future = asyncio.ensure_future(self._run_master_connect())
        server_future = asyncio.ensure_future(self._run_server_start())
        await master_future
        await server_future

    # master part -------------------------------------------------------------

    async def _run_master_connect(self):
        await self._master.connect(self._address, self._port)
        await self._master_connect_handshake()

    async def _master_connect_handshake(self):
        # Read and process the required three initialization packets.
        init_data = await self._master.read_packet()
        if init_data.type != 'version_init':
            raise ParseError(f'unexpected initialization packet: {init_data.type}')

        self._handle_version_init(
            msg.gz_string_pb2.GzString.FromString(init_data.serialized_data)
        )

        namespaces_data = await self._master.read_packet()

        # NOTE: This type string is mis-spelled in the official client
        # and server as of 2.2.1.  Presumably they'll just leave it be
        # to preserve network compatibility.
        if namespaces_data.type != 'topic_namepaces_init':
            raise ParseError(f'unexpected namespaces init packet: {namespaces_data.type}')

        self._handle_topic_namespaces_init(
            msg.gz_string_v_pb2.GzString_V.FromString(namespaces_data.serialized_data)
        )

        publishers_data = await self._master.read_packet()
        if publishers_data.type != 'publishers_init':
            raise ParseError(f'unexpected publishers init packet: {publishers_data.type}')

        self._handle_publishers_init(
            msg.publishers_pb2.Publishers.FromString(publishers_data.serialized_data)
        )

        logger.debug('Connection: initialized!')
        self._initialized = True

        asyncio.ensure_future(self._start_master_read_loop())
        return self

    @staticmethod
    def _handle_version_init(message):
        logger.debug(f'Manager.handle_version_init({message.data})')
        version = float(message.data.split(' ')[1])
        if version < 1.9:
            raise ParseError(f'Unsupported gazebo version: {message.data}')

    def _handle_topic_namespaces_init(self, message):
        self._namespaces = message.data
        logger.debug(f'Manager.handle_topic_namespaces_init: {str(self._namespaces)}')

    def _handle_publishers_init(self, message):
        logger.debug('Manager.handle_publishers_init')
        for publisher in message.publisher:
            self._publisher_records.add(PublisherRecord(publisher))
            logger.debug(f'  {publisher.topic} - {publisher.msg_type} {publisher.host}:{publisher.port}')

    async def _start_master_read_loop(self):
        # Enter the normal message dispatch loop.
        self._stop = False
        while not self._stop:
            message = await self._master.read_packet()
            if not self._stop:
                self._process_message(message)

    def _process_message(self, packet):
        logger.debug('Manager.process_message: ' + str(packet))
        if packet.type in Manager._MSG_HANDLERS:
            handler, packet_type = Manager._MSG_HANDLERS[packet.type]
            handler(self, packet_type.FromString(packet.serialized_data))
        else:
            logger.warn('unhandled message type: ' + packet.type)

    # server part -------------------------------------------------------------

    async def _run_server_start(self):
        await self._server.serve(self._handle_server_connection)

    async def _handle_server_connection(self, reader, writer):
        try:
            this_connection = connection.Connection("manager_client_n")
            this_connection.accept_connection(reader, writer)

            self._clients.append(this_connection)
            await self._read_server_data(this_connection)
        except Exception as e:
            print('Ex' + str(e))
            # logger.exception("Exception handling an incoming connection")

    async def _read_server_data(self, _connection):
        while not self._stop:
            message = await _connection.read_packet()
            if self._stop:
                return
            if message.type == 'sub':
                self._handle_server_sub(
                    _connection,
                    msg.subscribe_pb2.Subscribe.FromString(message.serialized_data)
                )
            else:
                logger.warning(f'Manager.handle_server_connection unknown msg: {message.type}')

    def _handle_server_sub(self, _connection, message):
        if message.topic not in self._publishers:
            logger.warning(f'Manager.handle_server_sub unknown topic: {message.topic}')
            return

        publisher = self._publishers[message.topic]
        if publisher.msg_type != message.msg_type:
            logger.error(f'Manager.handle_server_sub type mismatch '
                         f'requested={publisher.msg_type} publishing={message.msg_type}')
            return

        publisher.add_listener(_connection)

    def _handle_publisher_add(self, message):
        logger.debug(f'Manager.handle_publisher_add: {message.topic} - {message.msg_type} {message.host}:{message.port}')
        self._publisher_records.add(PublisherRecord(message))

    def _handle_publisher_del(self, message):
        logger.debug('Manager.handle_publisher_del:' + message.topic)
        try:
            self._publisher_records.remove(PublisherRecord(message))
        except KeyError:
            logger.debug('got publisher_del for unknown: ' + message.topic)

    def _handle_namespace_add(self, message):
        logger.debug('Manager.handle_namespace_add:' + message.data)
        self._namespaces.append(message.data)

    def _handle_publisher_subscribe(self, message):
        logger.debug('Manager.handle_publisher_subscribe:' + message.topic)
        logger.debug(f' our info: {self._server.listen_host}, {self._server.listen_port}')
        if message.topic not in self._subscribers:
            logger.debug('no subscribers!')
            return

        # Check to see if this is ourselves... if so, then don't do
        # anything about it.
        if ((message.host == self._server.listen_host and
             message.port == self._server.listen_port)):
            logger.debug('got publisher_subscribe for ourselves')
            return

        logger.debug(f'creating subscriber for: {message.topic} {message.host} {message.port}')

        subscriber = self._subscribers[message.topic]

        subscriber.subscribe(message)

    def _handle_unsubscribe(self, message):
        #TODO
        pass

    def _handle_unadvertise(self, message):
        #TODO
        pass

    _MSG_HANDLERS = {
        'publisher_add': (_handle_publisher_add, msg.publish_pb2.Publish),
        'publisher_del': (_handle_publisher_del, msg.publish_pb2.Publish),
        'namespace_add': (_handle_namespace_add, msg.gz_string_pb2.GzString),
        'publisher_subscribe': (_handle_publisher_subscribe,
                                msg.publish_pb2.Publish),
        'publisher_advertise': (_handle_publisher_subscribe,
                                msg.publish_pb2.Publish),
        'unsubscribe': (_handle_unsubscribe, msg.subscribe_pb2.Subscribe),
        'unadvertise': (_handle_unadvertise, msg.publish_pb2.Publish),
        }