from mavsdk.server_utility import StatusTextType

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.server_utility.send_status_text(StatusTextType.INFO, "Hello World!")

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
