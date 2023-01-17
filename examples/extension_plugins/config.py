from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Loading config from file: json")
    drone.config.from_file("examples/extension_plugins/config_files/config.json")

    drone.logger.info("-- Loading config from file: ini")
    drone.config.from_file("examples/extension_plugins/config_files/config.ini")

    drone.logger.info("-- Loading config from file: txt")
    drone.config.from_file("examples/extension_plugins/config_files/config.txt")

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
