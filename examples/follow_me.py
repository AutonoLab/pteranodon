from mavsdk.follow_me import Config

from pteranodon import SimpleDrone

default_height = 8.0  # in Meters
follow_distance = 2.0  # in Meters, this is the distance that the drone will remain away from Target while following it
# Direction relative to the Target
# Options are NONE, FRONT, FRONT_LEFT, FRONT_RIGHT, BEHIND
direction = Config.FollowDirection.BEHIND
responsiveness = 0.02


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("-- Arming")
    drone.action.arm()

    conf = Config(default_height, follow_distance, direction, responsiveness)
    drone.follow_me.set_config(conf)

    drone.logger.info("-- Taking off")
    drone.action.takeoff()
    drone.wait(8)

    drone.logger.info("-- Starting Follow Me Mode")
    drone.follow_me.start()
    drone.wait(8)

    drone.logger.info("-- Stopping Follow Me Mode")
    drone.follow_me.stop()
    drone.wait(8)

    drone.logger.info("-- Landing")
    drone.action.land()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
