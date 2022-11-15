from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    for status_text in drone.telemetry.status_text():
        print("Statustext:", status_text)

    drone.stop()


if __name__ == "__main__":
    run()
