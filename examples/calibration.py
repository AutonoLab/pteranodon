from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Starting gyroscope calibration")
    drone.calibration.calibrate_gyro()

    print("-- Starting accelerometer calibration")
    drone.calibration.calibrate_accelerometer()

    print("-- Starting magnetometer calibration")
    drone.calibration.calibrate_magnetometer()

    print("-- Starting board level horizon calibration")
    drone.calibration.calibrate_level_horizon()


if __name__ == "__main__":
    run()
