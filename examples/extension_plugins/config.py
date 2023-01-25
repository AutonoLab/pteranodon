from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Loading config from file: cfg")
    drone.config.from_file("examples/extension_plugins/config_files/config.cfg")

    drone.config.set_param("BAT1_N_CELLS", 3)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
