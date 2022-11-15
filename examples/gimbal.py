from mavsdk.gimbal import ControlMode, GimbalMode

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("Taking control of gimbal")
    drone.gimbal.take_control(ControlMode.PRIMARY)

    drone.logger.info("Setting gimbal mode")
    drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)

    drone.logger.info("Look forward first")
    drone.gimbal.set_pitch_and_yaw(0, 0)
    drone.wait(1)

    drone.logger.info("Look down")
    drone.gimbal.set_pitch_and_yaw(-90, 0)
    drone.wait(2)

    drone.logger.info("Back to horizontal")
    drone.gimbal.set_pitch_and_yaw(0, 0)
    drone.wait(2)

    drone.logger.info("Slowly look up")
    drone.gimbal.set_pitch_rate_and_yaw_rate(10, 0)
    drone.wait(3)

    drone.logger.info("Back to horizontal")
    drone.gimbal.set_pitch_and_yaw(0, 0)
    drone.wait(2)

    drone.logger.info("Look right")
    drone.gimbal.set_pitch_and_yaw(0, 90)
    drone.wait(2)

    drone.logger.info("Look forward again")
    drone.gimbal.set_pitch_and_yaw(0, 0)
    drone.wait(2)

    drone.logger.info("Slowly look to the left")
    drone.gimbal.set_pitch_rate_and_yaw_rate(0, -20)
    drone.wait(3)

    drone.logger.info("Look forward again")
    drone.gimbal.set_pitch_and_yaw(0, 0)
    drone.wait(2)

    drone.logger.info("Look at a ROI (region of interest)")
    drone.gimbal.set_roi_location(47.39743832, 8.5463316, 488)
    drone.wait(3)

    drone.logger.info("Look forward again")
    drone.gimbal.set_pitch_and_yaw(0, 0)
    drone.wait(2)

    drone.logger.info("Release control of gimbal again")
    drone.gimbal.release_control()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
