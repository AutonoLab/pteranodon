#!/usr/bin/env python3

import asyncio

import pygazebo
import pygazebo.msg.joint_cmd_pb2
import pygazebo.msg.world_stats_pb2

def callback(data):
    print("Received data")
    stats = pygazebo.msg.world_stats_pb2.WorldStatistics()
    stats = stats.FromString(data)
    print(stats)

async def subscriber(manager):
    subscriber = await manager.subscribe(
        '/gazebo/default/diagnostics',
        'gazebo.msgs.Worldstats',
        callback
    )
    print("created the subscriber")
    print("Waiting for connection")
    await subscriber.wait_for_connection()
    print("Waiting for data")
    while True:
        await asyncio.sleep(1)

async def publish_loop():
    manager = await pygazebo.connect()

    publisher = await manager.advertise(
        '/gazebo/default/pioneer2dx/joint_cmd',
        'gazebo.msgs.JointCmd'
    )

    sub_task = asyncio.ensure_future(subscriber(manager))

    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message.name = 'pioneer2dx::left_wheel_hinge'
    message.force = 100
    message.axis = 3
    print(message)
    while True:
        print("sending message")
        await publisher.publish(message)
        await asyncio.sleep(1)


loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())
