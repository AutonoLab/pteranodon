import asyncio
from time import time
from Interfaces.DroneInterface import DroneInterface as di

# Concrete implemention of DroneInterface using HexSoon edu 450

BAUDRATE = "9600" # TODO: verify baudrate
ADDRESS = "serial://" + ID + ":" + BAUDRATE

class HexSoon(di):

    def __init__(self) -> None:
        super().__init__()
        self._serialConnection = UARTBridge()
        self._serialConnection.initializeBridge()

    # connect hexsoon to uart, and if successful,
    # connect mavsdk to flight controller
    async def connect(self):
        try:
            await self._drone.connect(ADDRESS)
        except:
            print("unable to connect mavsdk")
        
if __name__ == "__main__":
    drone = HexSoon()

    loop = asyncio.get_event_loop()

    loop.run_until_complete(drone.connect())
    time.sleep(5)
    loop.run_until_complete(drone.arm())
    loop.run_until_complete(drone.takeoff())
    time.sleep(10)
    loop.run_until_complete(drone.startOffboard())
    time.sleep(5)
    loop.run_until_complete(drone.maneuverWithNED(10, 10, 0))
    time.sleep(10)
    loop.run_until_complete(drone.stopOffboard())

    

