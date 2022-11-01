import asyncio
from struct import pack

import pygazebo
from pygazebo.msg import imu_pb2
from pygazebo.msg import packet_pb2


def image_callback(data):
    print(f"Received Data of type: {type(data)}")
    # packet = imu_pb2.IMU.FromString(data)
    packet = packet_pb2.Packet.FromString(data)
    print(f"Decoded packet to: {type(packet)}")
    print(dir(packet))
    print(f"Packet timnestamp: {packet.stamp}")
    print(f"Packet type: {packet.type}")
    print(f"Packet serialized data: {packet.serialized_data}")

    imu_packet = imu_pb2.IMU.FromString(packet.serialized_data)
    print(dir(imu_packet))

async def publish_loop():
    print(dir(imu_pb2))
    print(dir(imu_pb2.IMU))
    manager = await pygazebo.connect()

    subscriber: pygazebo.Subscriber = await manager.subscribe(
        '/gazebo/default/iris/imu',
        'gazebo.msg.IMU',
        image_callback
    )
    print("Created subscriber")
    subscriber.wait_for_connection()

    print("Starting loop")
    while True:
        await asyncio.sleep(1)


loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())