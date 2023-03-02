#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .manager import Manager


async def connect(address: str = '127.0.0.1', port: int = 11345):
    """Create a connection to the Gazebo server.

    The Manager instance creates a connection to the Gazebo server,
    then allows the client to either advertise topics for publication,
    or to listen to other publishers.

    :param address: destination TCP server address
    :param port: destination TCP server port
    :returns: a Future indicating when the connection is ready
    """
    manager = Manager(address, port)
    await manager.start()
    return manager
