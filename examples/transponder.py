from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Getting transponder")
    drone.transponder.transponder()

    drone.wait_until_queue_empty()
    drone.stop()

    drone.logger_info(drone.transponder.transponder)


if __name__ == "__main__":
    run()
    