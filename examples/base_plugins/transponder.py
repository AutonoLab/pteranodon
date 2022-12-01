from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Getting transponder")
    drone.logger.info(drone.transponder.transponder)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
