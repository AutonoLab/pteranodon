from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    for flight_mode in drone.telemetry.flight_mode():
        drone.logger.info("FlightMode:", flight_mode)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
    