from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("--- View battery")
    drone.telemetry.battery()

    print("--- View gps info")
    drone.telemetry.gps_info()

    print("--- View telemetry in air")
    drone.telemetry.in_air()

    print("--- View position")
    drone.telemetry.position()


if __name__ == "__main__":
    run()