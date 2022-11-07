import asyncio
import base64

import cv2
import numpy as np

import pygazebo


def image_callback(data):
    print(f"Received Data of type: {type(data)}")
    data = pygazebo.msg.image_stamped_pb2.ImageStamped.FromString(data)
    # print(dir(data))
    # for idx, elem in enumerate(data.image):
    #     print(f"{idx}: {type(elem)}")
    # print(type(data.time))
    try:
        img_pb2 = data.image
        height = img_pb2.height
        width = img_pb2.width
        img_time = data.time
        
        img = np.zeros((img_pb2.height, img_pb2.width), dtype=int)
        # img_pb2 = str(img_pb2.data).strip()[2:-1].split("\\")
        
        print(dir(img_pb2))
        print(f"Height: {img_pb2.height}")
        print(f"Width: {img_pb2.width}")
        print(f"Step: {img_pb2.step}")
        print(f"Pixel_format: {img_pb2.pixel_format}")

        # convert a string of bytes of type RBG_FLOAT16 with a step size to a numpy array
        img = np.fromstring(img_pb2.data, dtype=np.float16)
        img = np.uint8(img * 255)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)

        print(img)

        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # # convert std_msgs:Image to opencV image
        # try:
        #     img = np.frombuffer(img_pb2.data, dtype=np.float16)
        #     img = np.reshape(img, (height, width))
        #     # img = np.uint8(img * 255)
        #     print(img.shape)
        #     print(img.dtype)
        #     print(img)
        #     cv2.imshow("Depth", img)
        #     cv2.waitKey(1)
        # except Exception as e:
        #     print(e)

        # # iterate over all elements
        # print("processing an image")
        # row, col  = 0, 0
        # for b in img_pb2:
        #     if b == "x7f" or b == "":
        #         continue
        #     img[row][col] = int(b[1:2], 16)
        #     if col >= width:
        #         row += 1
        #         col = 0
        # print("DONE")

        cv2.imshow("testing", img)
        cv2.waitKey(0)

    except Exception as e:
        print(e)


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