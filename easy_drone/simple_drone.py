from time import sleep

from .abstract_drone import AbstractDrone

class SimpleDrone(AbstractDrone):
    def __init__(self, address: str):
        super().__init__(address=address)

    def setup(self):
        pass

    def loop(self):
        print("Loop running")
        time.sleep(1)

    def teardown(self):
        pass


if __name__ == "__main__":
    address = input("Enter the address to connect to the drone with, enter . for default linux serial")
    if address == ".":
        address = "serial:///dev/ttyACM0"

    drone = Simple(address)
    drone.arm()
    drone.takeoff()

    time.sleep(10)

    drone.land()
    drone.stop()
