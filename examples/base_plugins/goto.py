from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.telemetry.health()

    drone.telemetry.home()

    drone.logger.info("-- Arming")
    drone.arm()

    drone.logger.info("-- Taking off")
    drone.takeoff()
    drone.wait(1)

    flying_alt = 20.0
    drone.put(drone.action.goto_location, 47.397606, 8.543060, flying_alt, 0)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
