from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")


def send(drone, command):
    drone.shell.send(command)


if __name__ == "__main__":
    run()


