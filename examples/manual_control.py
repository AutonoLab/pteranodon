from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Taking off")
    drone.put(drone.action.takeoff)
    drone.wait(5)

    drone.logger.info("-- Starting manual control")
    drone.manual_control.start_position_control()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
