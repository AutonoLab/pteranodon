from abc import abstractclassmethod
from mavsdk import System
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)
from mavsdk import telemetry
# General interface of drone hardware subsystems
# Implement with your concrete drone model and sensors

class DroneInterface():
    
    def __init__(self) -> None:
        self._drone = System()

    ## METHODS TO OVERRIDE ##

    # override this function to meet your specific 
    # mavsdk device connection requirements
    @abstractclassmethod
    async def connect(self):
        pass

    ## END METHODS TO OVERRIDE ##

    # the following functions are standard across connection types
    
    async def arm(self):
        try:
            await self._drone.action.arm()
        except:
            print("unable to arm")

    async def takeoff(self):
        try:
            await self._drone.action.takeoff()
        except:
            print("unable to takeoff")

    async def land(self):
        try:
            await self._drone.action.land()
        except:
            print("unable to land")

    async def updateTracker(self, lat, long, elev):
        adversaryLocation = TargetLocation(lat, long, 1, 1, 1, 1) 
        try:
            await self._drone.follow_me.set_target_location(adversaryLocation)
        except:
            print('unable to update target location') 

    async def configureTracker(self):
        config = Config(10, 1, 1, 1.0)
        try:
            await self._drone.follow_me.set_config(config)
        except:
            print('unable to set config')

    async def startTracker(self):
        try:
            await self._drone.follow_me.start()
        except:
            print('unable to start tracker')

    async def stopTracker(self):
        try:
            await self._drone.follow_me.stop()
        except:
            print('unable to stop tracker')

    async def getLocation(self):
        try:
            location = self._drone.telemetry.gps_info()
            return location
        except: 
            print('unable to get coordinates')


    async def getBatteryLevel(self):
        try:
            batteryLevel = self._drone.telemetry.battery()
            return batteryLevel
        except: 
            print('unable to get coordinates')    

    async def getCameraFrame():
        pass #TODO

    async def disconnect(self):
        pass