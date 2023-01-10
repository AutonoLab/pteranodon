from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Starting gyroscope calibration")
    drone.calibration.calibrate_gyro()

    drone.logger.info("-- Starting accelerometer calibration")
    drone.calibration.calibrate_accelerometer()

    drone.logger.info("-- Starting magnetometer calibration")
    drone.calibration.calibrate_magnetometer()

    drone.logger.info("-- Starting board level horizon calibration")
    drone.calibration.calibrate_level_horizon()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
