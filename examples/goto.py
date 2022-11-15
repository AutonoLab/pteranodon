from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.telemetry.health()

    drone.telemetry.home()

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Taking off")
    drone.action.takeoff()

    drone.wait()

    drone.stop()


if __name__ == "__main__":
    run()
    