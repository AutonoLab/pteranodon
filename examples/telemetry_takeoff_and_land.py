from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Taking off")
    drone.put(drone.action.set_takeoff_altitude, 10.0)
    drone.action.takeoff()
    drone.wait(10)

    drone.logger.info("-- Landing")
    drone.put(drone.action.land)

    drone.telemetry.register_position_handler(print)

    drone.telemetry.register_flight_mode_handler(print)

    drone.logger.info(drone.telemetry.in_air)
    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()

