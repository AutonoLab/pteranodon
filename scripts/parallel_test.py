from pteranodon import SimpleDrone

for count in range(100):
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Taking off")
    drone.action.takeoff()
    drone.wait(10)

    drone.logger.info("-- Landing")
    drone.action.land()

    drone.wait_until_queue_empty()
    drone.stop()
    drone.wait(20)
