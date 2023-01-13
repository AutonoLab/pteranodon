from mavsdk.offboard import VelocityBodyYawspeed

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Setting initial setpoint")
    drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    drone.logger.info("-- Starting offboard")
    drone.offboard.start()

    drone.logger.info("-- Turn clock-wise and climb")
    drone.put(
        drone.offboard.set_velocity_body, VelocityBodyYawspeed(0.0, 0.0, -1.0, 60.0)
    )
    drone.wait(5)

    drone.logger.info("-- Turn back anti-clockwise")
    drone.put(
        drone.offboard.set_velocity_body, VelocityBodyYawspeed(0.0, 0.0, 0.0, -60.0)
    )
    drone.wait(5)

    drone.logger.info("-- Wait for a bit")
    drone.put(
        drone.offboard.set_velocity_body, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    drone.wait(2)

    drone.logger.info("-- Fly a circle")
    drone.put(
        drone.offboard.set_velocity_body, VelocityBodyYawspeed(5.0, 0.0, 0.0, 30.0)
    )
    drone.wait(15)

    drone.logger.info("-- Wait for a bit")
    drone.put(
        drone.offboard.set_velocity_body, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    drone.wait(5)

    drone.logger.info("-- Fly a circle sideways")
    drone.put(
        drone.offboard.set_velocity_body, VelocityBodyYawspeed(0.0, -5.0, 0.0, 30.0)
    )
    drone.wait(15)

    drone.logger.info("-- Wait for a bit")
    drone.put(
        drone.offboard.set_velocity_body, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    drone.wait(8)

    drone.logger.info("-- Stopping offboard")
    drone.put(drone.offboard.stop)  # not sure if these need to be in a drone.put

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
