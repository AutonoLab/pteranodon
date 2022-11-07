from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("-- Getting transponder")
    drone.transponder.transponder()


if __name__ == "__main__":
    run()
