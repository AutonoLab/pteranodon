from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("Setting mode to 'PHOTO'")
    drone.camera.set_mode(drone.camera.mode.PHOTO)

    drone.logger.info("Taking a photo")
    drone.camera.take_photo()

    drone.stop()


if __name__ == "__main__":
    run()
