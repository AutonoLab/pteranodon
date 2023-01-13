from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("Getting camera mode")
    drone.logger.info(f"Current camera mode: {drone.camera.mode}")

    drone.logger.info("Get current camera settings")
    drone.logger.info(f"Current camera settings: {drone.camera.current_settings}")

    drone.logger.info("Display possible setting options")
    drone.logger.info(f"Possible camera settings: {drone.camera.possible_settings_options}")

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
