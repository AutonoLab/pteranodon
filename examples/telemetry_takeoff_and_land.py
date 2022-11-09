from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Arming")
    drone.action.arm()

    print("-- Taking off")
    drone.action.set_takeoff_altitude(10.0)
    drone.action.takeoff()

    print("-- Landing")
    drone.action.land()


if __name__ == "__main__":
    run()

