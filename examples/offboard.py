from mavsdk.offboard import Attitude

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Arming")
    drone.action.arm()

    print("-- Setting initial setpoint")
    drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    print("-- Go up at 70% thrust")
    drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.7))

    print("-- Roll 30 at 60% thrust")
    drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))

    print("-- Roll -30 at 60% thrust")
    drone.offboard.set_attitude(Attitude(-30.0, 0.0, 0.0, 0.6))

    print("-- Hover at 60% thrust")
    drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))


if __name__ == "__main__":
    run()
