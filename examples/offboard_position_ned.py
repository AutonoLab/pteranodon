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

    print("-- Go 0m North, 0m East, -5m Down within local coordinate system")
    drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))

    print("-- Go 5m North, 0m East, -5m Down within local coordinate system, turn to face East")
    drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, -5.0, 90.0))

    print("-- Go 5m North, 10m East, -5m Down within local coordinate system")
    drone.offboard.set_position_ned(PositionNedYaw(5.0, 10.0, -5.0, 90.0))

    print("-- Go 0m North, 10m East, 0m Down within local coordinate system, turn to face South")
    drone.offboard.set_position_ned(PositionNedYaw(0.0, 10.0, 0.0, 180.0))

    print("-- Stopping offboard")
    drone.offboard.stop()


if __name__ == "__main__":
    run()

