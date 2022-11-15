from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.stop()


if __name__ == "__main__":
    run()
