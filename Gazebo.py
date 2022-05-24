from DroneInterface import DroneInterface as di

# Concrete implemention of DroneInterface using HexSoon edu 450

URL_TYPE = "udp" # can be udp or tcp
IP = "0.0.0.0" # ip of Gazebo instance
PORT = "11111" # port to Gazebo instance

ADDRESS = URL_TYPE + "://" + IP + ":" + PORT

class Gazebo(di):

    def __init__(self) -> None:
        super().__init__()

    # connect hexsoon to uart, and if successful,
    # connect mavsdk to flight controller
    async def connect(self):
        try:
            await self._drone.connect(ADDRESS)
        except:
            print("unable to connect mavsdk")