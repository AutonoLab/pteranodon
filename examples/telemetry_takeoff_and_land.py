from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Taking off")
    drone.action.set_takeoff_altitude(10.0)
    drone.action.takeoff()
    drone.wait(10)

    drone.logger.info("-- Landing")
    drone.action.land()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()

