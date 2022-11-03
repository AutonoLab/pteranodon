from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Arming")
    drone.action.arm()

    print("-- Taking off")
    drone.action.takeoff()

    print("-- Starting Follow Me Mode")
    drone.follow_me.start()

    print("-- Stopping Follow Me Mode")
    drone.follow_me.stop()

    print("-- Landing")
    drone.action.land()


if __name__ == "__main__":
    run()

