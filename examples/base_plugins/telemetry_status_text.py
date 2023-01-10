from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info(drone.telemetry.status_text)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
