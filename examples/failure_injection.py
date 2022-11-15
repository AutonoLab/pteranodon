from mavsdk.failure import FailureUnit, FailureType

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Enabling failure injection")
    drone.param.set_param_int('SYS_FAILURE_EN', 1)

    drone.logger.info("-- Injecting GPS failure")
    drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OFF, instance=0)

    drone.logger.info("-- Disabling failure injection")
    drone.param.set_param_int('SYS_FAILURE_EN', 0)

    drone.stop()


if __name__ == "__main__":
    run()
