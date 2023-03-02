========
pygazebo
========

.. image:: https://travis-ci.org/justincdavis/pygazebo.png?branch=main
        :target: https://travis-ci.org/justincdavis/pygazebo

.. image:: https://coveralls.io/repos/justincdavis/pygazebo/badge.png?branch=main
       :target: https://coveralls.io/r/justincdavis/pygazebo?branch=main

pygazebo provides python bindings for the Gazebo
(http://gazebosim.org) multi-robot simulator.

* Original GitHub project: https://github.com/jpieper/pygazebo
* Free software: Apache 2.0 License

Features
--------

* Supports publishing and subscribing to any Gazebo topics using a
  straightforward python API.
* Python versions of all defined Gazebo protobuf messages are
  included.
* Based on asyncio for flexible concurrency support.

Simple Usage
------------

The following example shows how easy it is to publish a message
repeatedly to control a single joint in a Gazebo model running on the
local machine on the default port.

.. code-block:: python
  
  import asyncio
  
  import pygazebo
  import pygazebo.msg.joint_cmd_pb2
  
  
  async def publish_loop():
      manager = await pygazebo.connect()
      
      publisher = await manager.advertise('/gazebo/default/model/joint_cmd',
                                          'gazebo.msgs.JointCmd')
  
      message = pygazebo.msg.joint_cmd_pb2.JointCmd()
      message.name = 'robot::joint_name'
      message.axis = 0
      message.force = 1.0

      while True:
          await publisher.publish(message)
          await asyncio.sleep(1.0)
  
  loop = asyncio.get_event_loop()
  loop.run_until_complete(publish_loop())
