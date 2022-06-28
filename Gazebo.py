from Interfaces.DroneInterface import DroneInterface as di
import asyncio
import time
import atexit
# Concrete implemention of DroneInterface using HexSoon edu 450

URL_TYPE = "udp" # can be udp or tcp
IP = "" # ip of Gazebo instance
PORT = "14540" # port to Gazebo instance

ADDRESS = URL_TYPE + "://" + IP + ":" + PORT

class Gazebo(di):

	def __init__(self) -> None:
		super().__init__()

    # connect mavsdk to Gazebo
	async def connect(self):
		try:
			await self._drone.connect(ADDRESS)
			return True
		except:
			print("unable to connect mavsdk")
			return False


if __name__ == "__main__":
        drone = Gazebo()
        loop = asyncio.get_event_loop()

        # drone.connect()

        # loop.run_until_complete(drone.connect())
        
        drone = drone.start()

        # Set NAV_RCL_ACT to 0
        loop.run_until_complete(drone._drone.param.set_param_int("COM_RCL_EXCEPT", 4))

        print("Sleep")
        time.sleep(5)
    
        loop.run_until_complete(drone._drone.action.takeoff())
        print('takeoff above')

        time.sleep(10)

        loop.run_until_complete(drone.startOffboard())
        time.sleep(5)
        print('forward')
        loop.run_until_complete(drone.maneuverWithNED(10, 0, 0))
        time.sleep(10)
        print('back')
        loop.run_until_complete(drone.maneuverWithNED(-10, 0, 0))
        time.sleep(10)
        print('right')
        loop.run_until_complete(drone.maneuverWithNED(0, 10, 0))
        time.sleep(10)
        print('back')
        loop.run_until_complete(drone.maneuverWithNED(-10, 0, 0))
        time.sleep(10)
        print('stopping offboard')
        loop.run_until_complete(drone.stopOffboard())
