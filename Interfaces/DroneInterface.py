from abc import abstractclassmethod
from turtle import forward
from mavsdk import System
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)
from mavsdk import telemetry
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed
from math import atan, degrees, sqrt, pow, cos, sin, radians
import logging

class DroneInterface():
    
    def __init__(self) -> None:
        self._drone = System()
        self._camera = None
        logging.basicConfig(
            filename="mavLog.log",
            filemode="w",
            format="%(levelname)s %(asctime)s - %(message)s",
            level=logging.ERROR
        )
        self.logger = logging.getLogger()

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
            return True
        except Exception as e:
            self.logger.error(e)
            return False
            

    async def takeoff(self):
        try:
            await self._drone.action.takeoff()
            return True
        except Exception as e:
            self.logger.error(e)
            return False

    async def land(self):
        try:
            await self._drone.action.land()
            return True
        except Exception as e:
            self.logger.error(e)
            return False

    async def updateTracker(self, lat, long, elev):
        adversaryLocation = TargetLocation(lat, long, 1, 1, 1, 1) 
        try:
            await self._drone.follow_me.set_target_location(adversaryLocation)
            return True
        except Exception as e:
            self.logger.error(e)
            return False

    async def configureTracker(self):
        config = Config(10, 1, 1, 1.0)
        try:
            await self._drone.follow_me.set_config(config)
            return True
        except Exception as e:
            self.logger.error(e)
            return False

    async def startTracker(self):
        try:
            await self._drone.follow_me.start()
        except:
            print('unable to start tracker')

    async def stopTracker(self):
        try:
            await self._drone.follow_me.stop()
            return True
        except Exception as e:
            self.logger.error(e)
            return False

    async def getLocation(self):
        try:
            location = self._drone.telemetry.gps_info()
            return location
        except Exception as e:
            self.logger.error(e)
            return False

    async def getPositionNED(self):
        try:
            position = self._drone.telemetry.position_velocity_ned()
            async for pos in position:
                old_pos = pos
                break
            return old_pos
        except Exception as e:
            self.logger.error(e)
            return False

    async def getBatteryLevel(self):
        try:
            batteryLevel = self._drone.telemetry.battery()
            return batteryLevel
        except Exception as e:
            self.logger.error(e)
            return False 

    def getCameraFrame(self):
        frame = self._camera.getFrame()
        return frame

    async def disconnect(self):
        pass

    async def maneuverTo(self, frame, cnn_x, cnn_y):
        localCoordinates = self._camera.deprojectPixelToPoint(frame, cnn_x, cnn_y)
        Front = localCoordinates[2]
        Right = localCoordinates[0]
        Down = 0 - localCoordinates[1]
        
        return await self.maneuverWithNED(Front, Right, Down)

    async def getAngle(self):
        try:
            angle = self._drone.telemetry.attitude_euler()
            async for a in angle:
                old_a = a
                break
            return old_a
        except Exception as e:
            self.logger.error(e)
            return False

    async def maneuverWithNED(self, Front, Right, Down):
        
        # get current position
        task = await self.getPositionNED()
        currentPos = task.position
        print(currentPos)
        
        # get angle of rotation
        task2 = await self.getAngle()
        angle = task2.yaw_deg
        angleOfRotation = radians(angle)
        print(angle)

        # convert FRD to NED 
        North = Right*sin(angleOfRotation) + Front*cos(angleOfRotation)
        East = Right*cos(angleOfRotation) - Front*sin(angleOfRotation)

        # add offset to curent position
        North = North + currentPos.north_m
        East = East + currentPos.east_m
        Down = Down + currentPos.down_m

        # get angle of yaw
        if ( North == 0 ) and ( East > 0 ):
            Yaw = 90
        elif ( North == 0 ) and ( East < 0 ):
            Yaw = -90
        else:
            Yaw = degrees( atan( East / North ) )

        newPos = PositionNedYaw(North, East, Down, Yaw)
        print(newPos)
        try:
            await self._drone.offboard.set_position_ned(newPos)
            return True
        except Exception as e:
            self.logger.error(e)
            return False
        

    async def setHeadingNED(self, Forward, Right, Down):
        targetSpeed = 5 # fixed speed of 5ms

        totalDistance = sqrt( pow(Forward, 2) + pow(Right, 2) + pow(Down, 2) )
        targetTime = totalDistance / targetSpeed
        Forward_ms = Forward / targetTime
        Right_ms = Right / targetTime
        Down_ms = - Down / targetTime

        if Forward == 0:
            Yaw = 90
        else:
            Yaw = degrees(atan(Right / Forward))

        targetYawSpeed = Yaw / targetTime

        motionVector = VelocityBodyYawspeed(Forward_ms, Right_ms, Down_ms, 0)
        
        try:
            await self._drone.offboard.set_velocity_body(motionVector)
            return True
        except Exception as e:
            self.logger.error(e)
            return False

    async def offboardHold(self):
        try:
            await self._drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
            return True
        except Exception as e:
            self.logger.error(e)
            return False

    async def startOffboard(self):
        
        try:
            await self._drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
            try:
                await self._drone.offboard.start()
                return True
            except Exception as e:
                self.logger.error(e)
                return False
        except Exception as e:
            self.logger.error(e)
            return False


    async def stopOffboard(self):
        try:
            await self._drone.offboard.stop()
            return True
        except Exception as e:
            self.logger.error(e)
            return False


