from mavsdk.offboard import VelocityBodyYawspeed

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

    print("-- Turn clock-wise and climb")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1.0, 60.0))

    print("-- Turn back anti-clockwise")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, -60.0))

    print("-- Wait for a bit")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Fly a circle")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(5.0, 0.0, 0.0, 30.0))

    print("-- Wait for a bit")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Fly a circle sideways")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, -5.0, 0.0, 30.0))

    print("-- Wait for a bit")
    drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Stopping offboard")
    drone.offboard.stop()

    
if __name__ == "__main__":
    run()
