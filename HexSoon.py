from Interfaces.DroneInterface import DroneInterface as di
from UARTBridge import ID, UARTBridge

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
        connectionStatus = self._serialConnection.connectBridge()

        if(connectionStatus):
            try:
                await self._drone.connect(ADDRESS)
                # TODO: add baudrate to ID?
                # e.g. ID:baudrate
            except:
                print("unable to connect mavsdk")
        
        else:
            print("unable to connect serial")
        