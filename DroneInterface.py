from abc import abstractclassmethod
from mavsdk import System

# General interface of drone hardware subsystems
# Implement with your concrete drone model and sensors

class DroneInterface():
    
    ## METHODS TO OVERRIDE ##

    # override this method to include your bridge connection type
    def __init__(self) -> None:
        self._drone = System()

    # override this function to meet your specific 
    # mavsdk device connection requirements
    @abstractclassmethod
    async def connect(self):
        pass

    ## END METHODS TO OVERRIDE ##

    # the following functions are standard across connection types
    
    async def takeoff(self):
        try:
            await self._drone.action.arm()
        except:
            print("unable to arm")

        try:
            await self._drone.action.takeoff()
        except:
            print("unable to takeoff")

    async def land(self):
        await self._drone.action.land()