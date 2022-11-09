from mavsdk.offboard import VelocityBodyYawspeed, VelocityNedYaw

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Arming")
    drone.action.arm()

    print("-- Setting initial setpoint")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    drone.offboard.start()

    print("-- Go up 2 m/s")
    drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))

    print("-- Go North 2 m/s, turn to face East")
    drone.offboard.set_velocity_ned(VelocityNedYaw(2.0, 0.0, 0.0, 90.0))

    print("-- Go South 2 m/s, turn to face West")
    drone.offboard.set_velocity_ned(
        VelocityNedYaw(-2.0, 0.0, 0.0, 270.0))

    print("-- Go West 2 m/s, turn to face East")
    drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -2.0, 0.0, 90.0))

    print("-- Go East 2 m/s")
    drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 2.0, 0.0, 90.0))

    print("-- Turn to face South")
    drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 180.0))

    print("-- Go down 1 m/s, turn to face North")
    drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 1.0, 0.0))

    print("-- Stopping offboard")
    drone.offboard.stop()


if __name__ == "__main__":
    run()
