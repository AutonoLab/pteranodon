from mavsdk.offboard import PositionNedYaw

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Arming")
    drone.action.arm()

    print("-- Setting initial setpoint")
    drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    drone.offboard.start()

    print("-- Go 0m North, 0m East, -10m Down within local coordinate system")
    # await drone.offboard.set_position_velocity_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0),VelocityNedYaw(0.0,0.0,
    # -1.0,0.0))
    drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))

    print("-- Go 10m North, 0m East, 0m Down within local coordinate system")
    drone.offboard.set_position_ned(PositionNedYaw(50.0, 0.0, -10.0, 0.0))
    # await drone.offboard.set_position_velocity_ned(PositionNedYaw(50.0, 0.0, -10.0, 0.0),VelocityNedYaw(1.0,0.0,
    # 0.0,0.0))

    drone.action.land()

    print("-- Stopping offboard")
    drone.offboard.stop()


if __name__ == "__name__":
    run()
