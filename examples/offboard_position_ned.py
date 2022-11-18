from mavsdk.offboard import PositionNedYaw

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Setting initial setpoint")
    drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    drone.logger.info("-- Starting offboard")
    drone.offboard.start()

    drone.logger.info("-- Go 0m North, 0m East, -5m Down within local coordinate system")
    drone.put(drone.offboard.set_position_ned, PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    drone.wait(10)

    drone.logger.info("-- Go 5m North, 0m East, -5m Down within local coordinate system, turn to face East")
    drone.put(drone.offboard.set_position_ned, PositionNedYaw(5.0, 0.0, -5.0, 90.0))
    drone.wait(10)

    drone.logger.info("-- Go 5m North, 10m East, -5m Down within local coordinate system")
    drone.put(drone.offboard.set_position_ned, PositionNedYaw(5.0, 10.0, -5.0, 90.0))
    drone.wait(15)

    drone.logger.info("-- Go 0m North, 10m East, 0m Down within local coordinate system, turn to face South")
    drone.put(drone.offboard.set_position_ned, PositionNedYaw(0.0, 10.0, 0.0, 180.0))
    drone.wait(10)

    drone.logger.info("-- Stopping offboard")
    drone.offboard.stop()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()

