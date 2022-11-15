from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Taking off")
    drone.action.takeoff()

    drone.logger.info("-- Starting manual control")
    drone.manual_control.start_position_control()

    drone.stop()


if __name__ == "__main__":
    run()
