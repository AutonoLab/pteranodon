from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("--- View battery")
    drone.logger.info(drone.telemetry.battery)

    drone.logger.info("--- View gps info")
    drone.logger.info(drone.telemetry.gps_info)

    drone.logger.info("--- View telemetry in air")
    drone.logger.info(drone.telemetry.in_air)

    drone.logger.info("--- View position")
    drone.logger.info(drone.telemetry.position)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
