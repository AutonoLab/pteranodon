#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Python bindings to the Gazebo multi-robot simulator
===================================================

This package provides a python API to interact with the Gazebo
multi-robot simulator, http://www.gazebosim.org.  Gazebo is a
multi-robot simulator for outdoor environments. Like Stage, it is
capable of simulating a population of robots, sensors and objects, but
does so in a three-dimensional world. It generates both realistic
sensor feedback and physically plausible interactions between objects
(it includes an accurate simulation of rigid-body physics).

pygazebo implements the Gazebo network publish-subscribe protocol, so
that python applications can seamlessly interact with Gazebo entities.

pygazebo is based on eventlet for asynchronous network operations.
"""

import logging
DEBUG_LEVEL = logging.WARNING

from .msg import gz_string_pb2
from .msg import gz_string_v_pb2
from .msg import packet_pb2
from .msg import publishers_pb2
from .msg import subscribe_pb2

from .subscriber import Subscriber
from .publisher import Publisher
from .pygazebo import Manager
from .pygazebo import connect

__all__ = ["connect", "Manager", "Publisher", "Subscriber"]

__author__ = 'Josh Pieper'
__email__ = 'jjp@pobox.com'
__version__ = '4.0.0-2019.07'
