from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.arm()

    drone.logger.info("-- Taking off")
    drone.put(drone.action.takeoff)
    drone.wait(10)

    drone.logger.info("-- Landing")
    drone.put(drone.action.land)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
