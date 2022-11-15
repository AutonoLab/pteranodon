from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    for is_armed in drone.telemetry.armed():
        drone.logger.info("Is_armed:", is_armed)

    for is_in_air in drone.telemetry.in_air():
        drone.logger.info("Is_in_air:", is_in_air)

    drone.stop()


if __name__ == "__main__":
    run()
