from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("--- View battery")
    drone.telemetry.battery()

    drone.logger.info("--- View gps info")
    drone.telemetry.gps_info()

    drone.logger.info("--- View telemetry in air")
    drone.telemetry.in_air()

    drone.logger.info("--- View position")
    drone.telemetry.position()

    drone.stop()


if __name__ == "__main__":
    run()
