from mavsdk.offboard import Attitude

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Setting initial setpoint")
    drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    drone.logger.info("-- Go up at 70% thrust")
    drone.put(drone.offboard.set_attitude, Attitude(0.0, 0.0, 0.0, 0.7))
    drone.wait(2)

    drone.logger.info("-- Roll 30 at 60% thrust")
    drone.put(drone.offboard.set_attitude, Attitude(30.0, 0.0, 0.0, 0.6))
    drone.wait(2)

    drone.logger.info("-- Roll -30 at 60% thrust")
    drone.put(drone.offboard.set_attitude, Attitude(-30.0, 0.0, 0.0, 0.6))
    drone.wait(2)

    drone.logger.info("-- Hover at 60% thrust")
    drone.put(drone.offboard.set_attitude, Attitude(0.0, 0.0, 0.0, 0.6))
    drone.wait(2)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()

