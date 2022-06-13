# built in libraries
import asyncio
from abc import abstractmethod, ABC
from math import atan, degrees, sqrt, pow, cos, sin, radians
import logging
from threading import Thread
from collections import deque
from typing import Union
import time

# 3rd part libs
from mavsdk import System
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)
from mavsdk import telemetry
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed


class DroneInterface(ABC):
    def __init__(self, mavlink_poll_rate: Union[float, None] = None) -> None:
        self._drone = System()
        self._camera = None
        logging.basicConfig(
            filename="mavLog.log",
            filemode="w",
            format="%(levelname)s %(asctime)s - %(message)s",
            level=logging.ERROR
        )
        self._logger = logging.getLogger()

        # attributes related to threaded mavlink commands
        self._loop = asyncio.get_event_loop()
        self._stopped = False

        # declare threads for dispatching mavlink commands and for doing the controller
        self._mavlink_queue = deque()
        self._mavlink_poll_rate = 1.0 / 60.0 if mavlink_poll_rate is None else 1.0 / mavlink_poll_rate
        self._mavlink_thread = Thread(name="MAVLINK", target=self._mavlink_dispatcher, args=(), kwargs={}, daemon=None)

    # METHODS TO OVERRIDE #
    # override this function to meet your specific
    # mavsdk device connection requirements
    # TODO maybe switch this to a public private pair, so _connect is overridden, but high level is the same
    @abstractmethod
    async def connect(self) -> bool:
        return False
    # END METHODS TO OVERRIDE #

    # method for handling startup of drone. After this, there should only be calls to the public api functions
    def start(self) -> 'DroneInterface':
        connect_success = self._loop.run_until_complete(self.connect())
        if not connect_success:
            self._logger.fatal("Drone failed to connect in DroneInterface.start, exiting...")
            raise Exception("Drone failed to connect")
        arm_success = self._loop.run_until_complete(self._arm())
        while not arm_success:
            self._logger.error("Arming failure in DroneInterface.start, retrying...")
            arm_success = self._loop.run_until_complete(self._arm())
        return self._start_thread()

    # the following functions are standard across connection types
    # Methods for thread targetings
    async def _mavlink_dispatcher(self):
        elapsed_time = 0.0
        while not self._stopped:
            try:
                # get initial time
                start_time = time.perf_counter()
                # get command (using deque as FIFO queue)
                command = self._mavlink_queue.popleft()
                self._loop.run_in_executor(None, command)
                # get final time
                elapsed_time = time.perf_counter() - start_time
            except IndexError:
                pass
            finally:
                sleep_dur = self._mavlink_poll_rate - elapsed_time
                if sleep_dur > 0.0:
                    time.sleep(sleep_dur)

    def _start_thread(self) -> 'DroneInterface':
        self._mavlink_thread.start()
        return self

    def stop(self) -> None:
        self._stopped = True

    # mavlink related methods
    # NOTE: the following 3 methods (arm, takeoff, land) were switched to a private public system since their actual
    # calls can be queued up into the threaded setup. We can later poll the deque for its length
    # (it has accurate length) and make a decision based on how many actions are stored in the queue. Or if there are
    # lots of actions we can have it skip the delay
    async def _arm(self):  # switched to public/private pair, but can probably be just public or private
        try:
            await self._drone.action.arm()
            return True
        except Exception as e:
            self._logger.error(e)
            return False

    def arm(self):
        self._mavlink_queue.append(self._arm)

    async def _takeoff(self):
        try:
            await self._drone.action.takeoff()
            return True
        except Exception as e:
            self._logger.error(e)
            return False

    def takeoff(self):
        self._mavlink_queue.append(self._takeoff)

    async def _land(self):
        try:
            await self._drone.action.land()
            return True
        except Exception as e:
            self._logger.error(e)
            return False

    def land(self):
        self._mavlink_queue.append(self._land)

    async def _getLocation(self):
        try:
            location = self._drone.telemetry.gps_info()
            return location
        except Exception as e:
            self._logger.error(e)
            return False

    async def _getPositionNED(self):
        try:
            position = self._drone.telemetry.position_velocity_ned()
            old_pos = None
            async for pos in position:
                old_pos = pos
                break
            return old_pos
        except Exception as e:
            self._logger.error(e)
            return False

    async def _getBatteryLevel(self):
        try:
            batteryLevel = self._drone.telemetry.battery()
            return batteryLevel
        except Exception as e:
            self._logger.error(e)
            return False

    # TODO, this should not be a function in the high level class. Should have a sensor setup which opens threads for
    # TODO: sensors and reads them. Sensors should be stored as a dictionary or list for iterating, Dictionary for
    # TODO: getting their data out with a consistent system
    def _getCameraFrame(self):
        frame = self._camera.getFrame()
        return frame

    async def disconnect(self):
        # NOTE added a join for MAVLINK thread
        self._mavlink_queue.clear()
        self._mavlink_thread.join()
        # TODO add other functionalities

    async def maneuverTo(self, frame, cnn_x, cnn_y):
        localCoordinates = self._camera.deprojectPixelToPoint(frame, cnn_x, cnn_y)
        Front = localCoordinates[2]
        Right = localCoordinates[0]
        Down = 0 - localCoordinates[1]

        totalDistance = sqrt( pow(Front,2) + pow(Right, 2) + pow(Down,2) )

        if totalDistance < 5:
            return await self.offboardHold()
        else:
            return await self.maneuverWithNED(Front, Right, Down)

    async def _getAngle(self):
        try:
            angle = self._drone.telemetry.attitude_euler()
            old_a = None
            async for a in angle:
                old_a = a
                break
            return old_a
        except Exception as e:
            self._logger.error(e)
            return False

    async def maneuverWithNED(self, Front, Right, Down):

        # get current position
        task = await self._getPositionNED()
        currentPos = task.position
        self._logger.info(currentPos)

        # get angle of rotation
        task2 = await self._getAngle()
        angle = task2.yaw_deg
        angleOfRotation = radians(angle)
        self._logger.info(angle)

        # convert FRD to NED 
        North = Right * sin(angleOfRotation) + Front * cos(angleOfRotation)
        East = Right * cos(angleOfRotation) - Front * sin(angleOfRotation)

        # add offset to curent position
        North = North + currentPos.north_m
        East = East + currentPos.east_m
        Down = Down + currentPos.down_m

        # get angle of yaw
        if (North == 0) and (East > 0):
            Yaw = 90
        elif (North == 0) and (East < 0):
            Yaw = -90
        else:
            Yaw = degrees(atan(East / North))

        newPos = PositionNedYaw(North, East, Down, Yaw)
        self._logger.info(newPos)
        try:
            await self._drone.offboard.set_position_ned(newPos)
            return True
        except Exception as e:
            self._logger.error(e)
            return False

    async def _setHeadingNED(self, Forward, Right, Down):
        targetSpeed = 5  # fixed speed of 5ms

        totalDistance = sqrt(pow(Forward, 2) + pow(Right, 2) + pow(Down, 2))
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
            self._logger.error(e)
            return False

    async def offboardHold(self):
        try:
            await self._drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            return True
        except Exception as e:
            self._logger.error(e)
            return False

    async def startOffboard(self):

        try:
            await self._drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            try:
                await self._drone.offboard.start()
                return True
            except Exception as e:
                self._logger.error(e)
                return False
        except Exception as e:
            self._logger.error(e)
            return False

    async def stopOffboard(self):
        try:
            await self._drone.offboard.stop()
            return True
        except Exception as e:
            self._logger.error(e)
            return False
