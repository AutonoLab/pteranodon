from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Arming")
    drone.action.arm()

    print("-- Taking off")
    drone.action.takeoff()

    print("-- Starting manual control")
    drone.manual_control.start_position_control()


if __name__ == "__main__":
    run()
