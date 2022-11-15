from mavsdk.failure import FailureUnit, FailureType

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Enabling failure injection")
    drone.param.set_param_int('SYS_FAILURE_EN', 1)

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Taking off")
    drone.action.takeoff()

    drone.wait(5)
    goto_lat = 0.0
    goto_lon = 0.0
    goto_alt = 0.0

    drone.logger.info("-- Flying up")
    flying_alt = goto_alt + 20.0  # To fly drone 20m above the ground plane
    drone.action.goto_location(goto_lat, goto_lon, flying_alt, 0)

    drone.wait(5)

    drone.logger.info("-- Injecting GPS failure")
    drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OFF, instance=0)

    drone.logger.info("-- Disabling failure injection")
    drone.param.set_param_int('SYS_FAILURE_EN', 0)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
