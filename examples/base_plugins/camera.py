from mavsdk.camera import Mode
from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("Setting mode to 'PHOTO'")
    drone.camera.set_mode(Mode.PHOTO)
    drone.wait(2)

    drone.logger.info("Taking a photo")
    drone.camera.take_photo()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
