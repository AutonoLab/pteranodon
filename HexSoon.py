from DroneInterface import DroneInterface as di
from UARTBridge import ID, UARTBridge

# Concrete implemention of DroneInterface using HexSoon edu 450

class HexSoon(di):

    def __init__(self) -> None:
        super().__init__()
        self._serialConnection = UARTBridge()
        self._serialConnection.initializeBridge()

    # connect hexsoon to uart, and if successful,
    # connect mavsdk to flight controller
    async def connect(self):
        self._serialConnection.connectBridge()

        if(self._serialConnection.isConnected):
            await self._drone.connect(port=ID)
            # TODO: add baudrate to ID?
            # e.g. ID:baudrate
        else:
            print("Unable to connect mavsdk to drone")
        