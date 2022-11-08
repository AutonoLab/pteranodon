import asyncio
import sys

import cv2
import numpy as np

import pygazebo


def get_cvimg_from_gz_depth_img(img_msg):
    return np.ndarray(shape=(img_msg.height, img_msg.width, 1),
                    dtype=np.uint16, buffer=img_msg.data)

def image_callback(data):
    data = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
    img_pb2 = data.image
    img = get_cvimg_from_gz_depth_img(img_pb2)
    cv2.imshow("testing", img)
    cv2.waitKey(1)


async def publish_loop():
    manager = await pygazebo.connect()

    subscriber: pygazebo.Subscriber = await manager.subscribe(
        '/gazebo/default/iris_depth_camera/depth_camera/link/depth_camera/image',
        'gazebo.msg.ImagesStamped',
        image_callback
    )
    subscriber.wait_for_connection()

    while True:
        await asyncio.sleep(1)


loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())