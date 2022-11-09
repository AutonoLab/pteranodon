from mavsdk.failure import FailureUnit, FailureType

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Enabling failure injection")
    drone.param.set_param_int('SYS_FAILURE_EN', 1)

    print("-- Injecting GPS failure")
    drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OFF, instance=0)

    print("-- Disabling failure injection")
    drone.param.set_param_int('SYS_FAILURE_EN', 0)


if __name__ == "__main__":
    run()
