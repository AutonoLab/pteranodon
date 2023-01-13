from mavsdk.offboard import VelocityBodyYawspeed, VelocityNedYaw

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Setting initial setpoint")
    drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    drone.logger.info("-- Starting offboard")
    drone.offboard.start()

    drone.logger.info("-- Go up 2 m/s")
    drone.put(drone.offboard.set_velocity_ned, VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
    drone.wait(4)

    drone.logger.info("-- Go North 2 m/s, turn to face East")
    drone.put(drone.offboard.set_velocity_ned, VelocityNedYaw(2.0, 0.0, 0.0, 90.0))
    drone.wait(4)

    drone.logger.info("-- Go South 2 m/s, turn to face West")
    drone.put(drone.offboard.set_velocity_ned, VelocityNedYaw(-2.0, 0.0, 0.0, 270.0))
    drone.wait(4)

    drone.logger.info("-- Go West 2 m/s, turn to face East")
    drone.put(drone.offboard.set_velocity_ned, VelocityNedYaw(0.0, -2.0, 0.0, 90.0))
    drone.wait(4)

    drone.logger.info("-- Go East 2 m/s")
    drone.put(drone.offboard.set_velocity_ned, VelocityNedYaw(0.0, 2.0, 0.0, 90.0))
    drone.wait(4)

    drone.logger.info("-- Turn to face South")
    drone.put(drone.offboard.set_velocity_ned, VelocityNedYaw(0.0, 0.0, 0.0, 180.0))
    drone.wait(2)

    drone.logger.info("-- Go down 1 m/s, turn to face North")
    drone.put(drone.offboard.set_velocity_ned, VelocityNedYaw(0.0, 0.0, 1.0, 0.0))
    drone.wait(4)

    drone.logger.info("-- Stopping offboard")
    drone.offboard.stop()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
