from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    # wait for telemetry to be present
    while drone.telemetry.health is None:
        drone.wait(0.1, command=False)

    health = drone.telemetry.health
    while not health.is_global_position_ok and not health.is_home_position_ok:
        health = drone.telemetry.health
        drone.wait(0.01, command=False)

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
