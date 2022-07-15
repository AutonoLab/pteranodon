from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed
from threading import Thread
from time import sleep, perf_counter
from collections import deque
import asyncio
import atexit
from math import atan, degrees, sqrt, pow, cos, sin, radians
from abc import abstractmethod, ABC


class Drone(ABC):
    def __init__(self, address: str, time_slice=0.050, min_follow_distance=5.0, bypass_com_rcl=True):
        # setup the instance fields
        self._stopped_mavlink = False
        self._stopped_loop = False
        self._time_slice = time_slice
        self._min_follow_distance = min_follow_distance
        self._bypass_com_rcl = bypass_com_rcl
        self._address = address

        # setup resources for drone control, mavsdk.System, deque, thread, etc..
        self._drone = System()
        self._queue = deque()
        self._loop = asyncio.get_event_loop()
        self._mavlink_thread = Thread(name="Mavlink-Thread", target=self._process_command_loop)
        self._mavlink_thread.start()

        # register the stop method so everything cleans up
        atexit.register(self.stop)

        # connect the drone
        self._connect()

        # after connection run setup, then loop, run teardown during cleanup phase
        self.setup()
        self._loop_thread = Thread(name="Loop-Thread", target=self._loop_loop)

    # properties for the class
    @property
    def address(self):
        return self._address

    @property
    def time_slice(self):
        return self._time_slice

    @time_slice.setter
    def time_slice(self, val: float):
        self._time_slice = val

    @property
    def min_follow_distance(self):
        return self._min_follow_distance

    @min_follow_distance.setter
    def min_follow_distance(self, dist: float):
        self._min_follow_distance = dist

    # AREA TO OVERRIDE
    # abstract methods which over classes must override
    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def loop(self):
        pass

    def _loop_loop(self): 
        while not self._stopped_loop:
            self.loop()

    def start_loop(self):
        self._loop_thread.start()

    @abstractmethod
    def teardown(self):
        pass

    ###############################################################################
    # internal methods for drone control, connection, threading of actions, etc..
    ###############################################################################
    def _connect(self):
        try:
            self._loop.run_until_complete(self._drone.connect(system_address=self.address))
            if self._bypass_com_rcl:
                self._loop.run_until_complete(self._drone.param.set_param_int("COM_RCL_EXCEPT", 4))
        except KeyboardInterrupt:
            self._cleanup()
            raise KeyboardInterrupt
        finally:
            pass  # todo, something?

    def _cleanup(self):
        self._drone.__del__()
        del self._drone

    def _process_command(self, com, args, kwargs):
        if com is None:
            pass  # just means it timed out without a new command
        elif asyncio.iscoroutinefunction(com):  # if it is an async function
            _ = self._loop.run_until_complete(com(*args, **kwargs))
        else:  # typical sync function
            _ = com(*args, **kwargs)

    # loop for processing mavlink commands. Uses the time slice determined in the contructor for its maximum rate. 
    def _process_command_loop(self):
        try:
            while not self._stopped_mavlink:
                start_time = perf_counter()
                try:
                    com, args, kwargs = self._queue.popleft()
                    if args is None:
                        args = []
                    if kwargs is None:
                        kwargs = {}
                    self._process_command(com, args, kwargs)
                except IndexError:
                    pass
                finally:
                    end_time = perf_counter() - start_time
                    if end_time < self._time_slice:
                        sleep(self._time_slice - end_time)
        finally:
            self._cleanup()

    # method to stop the thread for processing mavlink commands
    def stop(self):
        self._stopped_loop = True
        try:
            self._loop_thread.join()  # join the loop thread first since it most likely generates mavlink commands
        except RuntimeError:  # occurs if the loop_thread was never started with self.start_loop()
            pass

        # clear the queue and run teardown
        self._queue.clear()
        self.teardown()
        while len(self._queue) > 0:  # simple spin wait to ensure any mavlink commands from teardown are run
            sleep(self._time_slice + 0.01)
        
        # finally join the mavlink thread and stop it
        self._stopped_mavlink = True
        self._mavlink_thread.join()

    # method for queueing mavlink commands
    # TODO, implement a clear queue or priority flag. using deque allows these operations to be deterministic
    def put(self, obj, args, kwargs):
        self._queue.append((obj, args, kwargs))

    ##################################################
    # methods which implement the mavsdk System actions
    ##################################################
    # TODO, implement the flag for put in these methods
    def arm(self):
        self.put(self._drone.action.arm, None, None)

    def disarm(self):
        self.put(self._drone.action.disarm, None, None)

    def do_orbit(self):
        self.put(self._drone.action.do_orbit, None, None)

    def get_maximum_speed(self):
        return self._loop.run_until_complete(self._drone.action.get_maximum_speed())

    def get_return_to_launch_altitude(self):
        return self._loop.run_until_complete(self._drone.action.get_return_to_launch_altitude())

    def get_takeoff_altitude(self):
        return self._loop.run_until_complete(self._drone.action.get_takeoff_altitude())

    def goto_location(self, *args, **kwargs):
        self.put(self._drone.action.goto_location, args, kwargs)

    def hold(self):
        self.put(self._drone.action.hold, None, None)

    def kill(self):
        self.put(self._drone.action.kill, None, None)

    def land(self):
        self.put(self._drone.action.land, None, None)

    def reboot(self):
        self.put(self._drone.action.reboot, None, None)

    def return_to_launch(self):
        self.put(self._drone.action.return_to_launch, None, None)

    def set_actuator(self, *args, **kwargs):
        self.put(self._drone.action.set_actuator, args, kwargs)

    def set_current_speed(self, *args, **kwargs):
        self.put(self._drone.action.set_current_speed, args, kwargs)

    def set_maximum_speed(self, *args, **kwargs):
        self.put(self._drone.action.set_maximum_speed, args, kwargs)

    def set_return_to_launch_altitude(self, *args, **kwargs):
        self.put(self._drone.action.set_return_to_launch_altitude, args, kwargs)

    def set_takeoff_altitude(self, *args, **kwargs):
        self.put(self._drone.action.set_takeoff_altitude, args, kwargs)

    def shutdown(self):
        self.put(self._drone.action.shutdown, None, None)

    def takeoff(self):
        self.put(self._drone.action.takeoff, None, None)

    def terminate(self):
        self.put(self._drone.action.terminate, None, None)

    def transition_to_fixedwing(self):
        self.put(self._drone.action.transition_to_fixedwing, None, None)

    def transition_to_multicopter(self):
        self.put(self._drone.action.transition_to_multicopter, None, None)

    #########################################################
    # methods for the mavsdk System telemetry 
    #########################################################
    def get_gps_info(self):
        return self._loop.run_until_complete(self._drone.telemetry.gps_info())

    def get_position_velocity_ned(self):
        position = self._loop.run_until_complete(self._drone.telemetry.position_velocity_ned())
        old_pos = None
        for pos in position:
            old_pos = pos
            break
        return old_pos

    def get_battery(self):
        return self._loop.run_until_complete(self._drone.telemetry.battery())

    def get_attitude_euler(self):
        angle = self._loop.run_until_complete(self._drone.telemetry.attitude_euler())
        old_a = None
        for a in angle:
            old_a = a
            break
        return old_a

    #########################################################
    # methods for using offboard
    #########################################################
    def start_offboard(self):
        self.offboard_hold()
        self.put(self._drone.offboard.start, None, None)

    def stop_offboard(self):
        self.offboard_hold()
        self.put(self._drone.offboard.stop, None, None)

    def offboard_hold(self):
        self.put(self._drone.offboard.set_velocity_body, VelocityBodyYawspeed(0, 0, 0, 0), None)

    def offboard_set_velocity_body(self, *args, **kwargs):
        self.put(self._drone.offboard.set_velocity_body, args, kwargs)

    def offboard_set_position_ned(self, *args, **kwargs):
        self.put(self._drone.offboard.set_position_ned, args, kwargs)

    #########################################################
    # methods for maneuvering
    #########################################################
    def maneuver_to(self, *args, **kwargs):
        """
        param:: front -> float
        param:: right -> float
        param:: down -> float
        """
        self.put(self._maneuver_to, args, kwargs)

    async def _maneuver_to(self, front, right, down):
        totalDistance = sqrt(pow(front,2) + pow(right, 2) + pow(down,2))

        if totalDistance < self._min_follow_distance:
            return await self.offboard_hold()
        else:
            return await self._maneuver_with_ned(front, right, down)

    async def _maneuver_with_ned(self, front, right, down):
        # get current position
        task = await self._drone.telemetry.position_velocity_ned()
        current_pos = task.position

        # get angle of rotation
        task2 = await self._drone.telemetry.attitude_euler()
        angle = task2.yaw_deg
        angle_of_rotation = radians(angle)

        # convert FRD to NED 
        north = right * sin(angle_of_rotation) + front * cos(angle_of_rotation)
        east = right * cos(angle_of_rotation) - front * sin(angle_of_rotation)

        # add offset to curent position
        north = north + current_pos.north_m
        east = east + current_pos.east_m
        down = down + current_pos.down_m

        # get angle of yaw
        if (north == 0) and (east > 0):
            yaw = 90
        elif (north == 0) and (east < 0):
            yaw = -90
        else:
            yaw = degrees(atan(east / north))

        new_pos = PositionNedYaw(north, east, down, yaw)
        await self._drone.offboard.set_position_ned(new_pos)


