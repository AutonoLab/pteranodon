from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.wait_until_queue_empty()
    drone.stop()


def send(drone, command):
    drone.shell.send(command)


if __name__ == "__main__":
    run()
