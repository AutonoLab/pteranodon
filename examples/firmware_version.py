from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    info = drone.info.get_version()
    drone.logger.info(info)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
    