from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.shell.register_receive_handler()

    drone.shell.send("command")

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
