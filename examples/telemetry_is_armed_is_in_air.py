from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info(drone.telemetry.armed)

    drone.logger.info(drone.telemetry.in_air)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
