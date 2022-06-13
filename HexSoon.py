import asyncio
from time import time

from Interfaces.DroneInterface import DroneInterface as di


# Concrete implemention of DroneInterface using HexSoon edu 450
class HexSoon(di):
    ID = "dev/ttyACM0"
    BAUDRATE = "9600"  # TODO: verify baudrate
    ADDRESS = "serial:///" + ID

    def __init__(self) -> None:
        super().__init__()

    # connect hexsoon to uart, and if successful,
    # connect mavsdk to flight controller
    async def connect(self) -> bool:
        try:
            await self._drone.connect(system_address=f"{HexSoon.ADDRESS}")
            print("Waiting for drone to connect...")
            async for state in self._drone.core.connection_state():
                if state.is_connected:
                    print(f"-- Connected to drone!")
                    break
            return True
        except:
            print("unable to connect mavsdk")
            return False


if __name__ == "__main__":
    drone = HexSoon()
    loop = asyncio.get_event_loop()

    drone = drone.start()
    time.sleep(5)

    loop.run_until_complete(drone.startOffboard())
    time.sleep(5)

    loop.run_until_complete(drone.maneuverWithNED(10, 10, 0))
    time.sleep(10)

    loop.run_until_complete(drone.stopOffboard())

    

