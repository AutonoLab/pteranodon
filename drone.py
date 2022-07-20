from threading import Thread
from time import sleep, perf_counter
from collections import deque
import asyncio
import atexit
from math import atan, degrees, sqrt, pow, cos, sin, radians
from abc import abstractmethod, ABC

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed, Attitude, OffboardError
from mavsdk.action import ActionError

class Drone(ABC):
    def __init__(self, address: str, time_slice=0.050, min_follow_distance=5.0):
        # setup the instance fields
        self._stopped_mavlink = False
        self._stopped_loop = False
        self._time_slice = time_slice
        self._min_follow_distance = min_follow_distance
        self._address = address

        # setup telemetry fields
        # TODO, could link this up to the other telemetry through a dictionary where we use a method name
        # acquired automatically as the key. Then the properties would return self._telemetry_dict[method_name]
        self._telemetry_gps_info = None
        self._telemetry_position_velocity_ned = None
        self._telemetry_battery = None
        self._telemetry_attitude_euler = None

        # setup resources for drone control, mavsdk.System, deque, thread, etc..
        self._drone = System()
        self._queue = deque()
        self._loop = asyncio.get_event_loop()
        self._mavlink_thread = Thread(name="Mavlink-Thread", target=self._process_command_loop)

        # register the stop method so everything cleans up
        atexit.register(self.stop)

        # connect the drone
        self._connect()

        # after connection run setup, then initialize loop, run teardown during cleanup phase
        self.setup()
        self._loop_thread = Thread(name="Loop-Thread", target=self._loop_loop)

        # setup the futures for telemetry
        # TODO, I think there is a cleaner interface for handling these. Perhaps naming them all _get_telemetry to start
        # and then using dir() to identify the methods and using ensure future on those. Lots more telemetry to add
        # it would future proof the code a lot 
        self._get_gps_info_task = asyncio.ensure_future(self._get_gps_info())
        self._get_position_velocity_ned_task = asyncio.ensure_future(self._get_position_velocity_ned())
        self._get_battery_task = asyncio.ensure_future(self._get_battery())
        self._get_attitude_euler_task = asyncio.ensure_future(self._get_attitude_euler())

        # finally start the mavlink thread
        self._mavlink_thread.start()
        sleep(0.001)  # short sleep to ensure thread is started

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
        except KeyboardInterrupt:
            self._cleanup()
            raise KeyboardInterrupt
        finally:
            pass  # todo, something?

    def _cleanup(self):
        self._drone.__del__()
        del self._drone

    def _process_command(self, com, args, kwargs):
        # print(f"Printing in process_com:\n{args}\n{kwargs}\nDone printing in process_com")
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
                # TODO, perform logging on these exceptions
                except IndexError as e:
                    # this exception is expected since we expect the deque to not have elements sometimes
                    if str(e) != "pop from an empty deque":
                        # if the exception makes it here, it is unexpected
                        print(e)
                except ActionError as e:
                    print(e)
                except OffboardError as e:
                    print(e)
                finally:
                    end_time = perf_counter() - start_time
                    if end_time < self._time_slice:
                        sleep(self._time_slice - end_time)
        finally:
            self._cleanup()

    # method to stop the thread for processing mavlink commands
    def stop(self):
        # on a stop call put a disarm call in the empty queue
        self._queue.clear()
        self.disarm()

        # shutdown the any asyncgens that have been opened
        self._loop.run_until_complete(self._loop.shutdown_asyncgens())

        # stop execution of the loop
        self._stopped_loop = True
        try:
            self._loop_thread.join()  # join the loop thread first since it most likely generates mavlink commands
        except RuntimeError:  # occurs if the loop_thread was never started with self.start_loop()
            pass

        # run teardown (queue should be clean from the clear before disarm)
        self.teardown()
        while len(self._queue) > 0:  # simple spin wait to ensure any mavlink commands from teardown are run
            sleep(self._time_slice + 0.01)
        
        # finally join the mavlink thread and stop it
        self._stopped_mavlink = True
        self._mavlink_thread.join()

    # method for queueing mavlink commands
    # TODO, implement a clear queue or priority flag. using deque allows these operations to be deterministic
    def put(self, obj, *args, **kwargs):
        # print(f"Printing in put:\n{args}\n{kwargs}\nDone printing in put")
        self._queue.append((obj, args, kwargs))

    ##################################################
    # methods which implement the mavsdk System actions
    ##################################################
    # TODO, implement the flag for put in these methods
    def arm(self):
        self.put(self._drone.action.arm)

    def disarm(self):
        self.put(self._drone.action.disarm)

    def do_orbit(self):
        self.put(self._drone.action.do_orbit)

    def get_maximum_speed(self):
        return self._loop.run_until_complete(self._drone.action.get_maximum_speed())

    def get_return_to_launch_altitude(self):
        return self._loop.run_until_complete(self._drone.action.get_return_to_launch_altitude())

    def get_takeoff_altitude(self):
        return self._loop.run_until_complete(self._drone.action.get_takeoff_altitude())

    def goto_location(self, *args, **kwargs):
        self.put(self._drone.action.goto_location, *args, **kwargs)

    def hold(self):
        self.put(self._drone.action.hold)

    def kill(self):
        self.put(self._drone.action.kill)

    def land(self):
        self.put(self._drone.action.land)

    def reboot(self):
        self.put(self._drone.action.reboot)

    def return_to_launch(self):
        self.put(self._drone.action.return_to_launch)

    def set_actuator(self, *args, **kwargs):
        self.put(self._drone.action.set_actuator, *args, **kwargs)

    def set_current_speed(self, *args, **kwargs):
        self.put(self._drone.action.set_current_speed, *args, **kwargs)

    def set_maximum_speed(self, *args, **kwargs):
        self.put(self._drone.action.set_maximum_speed, *args, **kwargs)

    def set_return_to_launch_altitude(self, *args, **kwargs):
        self.put(self._drone.action.set_return_to_launch_altitude, *args, **kwargs)

    def set_takeoff_altitude(self, *args, **kwargs):
        self.put(self._drone.action.set_takeoff_altitude, *args, **kwargs)

    def shutdown(self):
        self.put(self._drone.action.shutdown)

    def takeoff(self):
        self.put(self._drone.action.takeoff)

    def terminate(self):
        self.put(self._drone.action.terminate)

    def transition_to_fixedwing(self):
        self.put(self._drone.action.transition_to_fixedwing)

    def transition_to_multicopter(self):
        self.put(self._drone.action.transition_to_multicopter)

    #########################################################
    # methods for the mavsdk System telemetry 
    #########################################################
    def _get_telemetry(self, prop):
        pass  # TODO, idea is to have this be a wrapper to check if a telemetry is null and execute some behavior

    async def _get_gps_info(self):
        async for gps_info in self._drone.telemetry.gps_info():
            if gps_info != self._telemetry_gps_info:
                self._telemetry_gps_info = gps_info

    @property
    def telemetry_gps_info(self):
        return self._telemetry_gps_info

    async def _get_position_velocity_ned(self):
        async for position in self._drone.telemetry.position_velocity_ned():
            if position != self._telemetry_position_velocity_ned:
                self._telemetry_position_velocity_ned = position

    @property
    def telemetry_position_velocity_ned(self):
        return self._telemetry_position_velocity_ned

    async def _get_battery(self):
        async for battery in self._drone.telemetry.battery():
            if battery != self._telemetry_battery:
                self._telemetry_battery = battery

    @property
    def telemetry_battery(self):
        return self._telemetry_battery

    async def _get_attitude_euler(self):
        async for attitude in self._drone.telemetry.attitude_euler():
            if attitude != self._telemetry_attitude_euler:
                self._telemetry_attitude_euler = attitude

    @property
    def telemetry_attitude_euler(self):
        return self._telemetry_attitude_euler

    #########################################################
    # methods for using offboard
    #########################################################
    def start_offboard(self):
        self.put(self._drone.offboard.set_attitude, Attitude(0.0, 0.0, 0.0, 0.0))
        self.offboard_hold()
        self.put(self._drone.offboard.start)

    def stop_offboard(self):
        self.offboard_hold()
        self.put(self._drone.offboard.stop)

    def offboard_hold(self):
        self.put(self._drone.offboard.set_velocity_body, VelocityBodyYawspeed(0, 0, 0, 0))

    def offboard_set_velocity_body(self, *args, **kwargs):
        self.put(self._drone.offboard.set_velocity_body, *args, **kwargs)

    def offboard_set_position_ned(self, *args, **kwargs):
        self.put(self._drone.offboard.set_position_ned, *args, **kwargs)

    #########################################################
    # methods for maneuvering
    #########################################################
    def maneuver_to(self, *args, **kwargs):
        """
        param:: front -> float
        param:: right -> float
        param:: down -> float
        """
        self.put(self._maneuver_to, *args, **kwargs)

    async def _maneuver_to(self, front, right, down):
        totalDistance = sqrt(pow(front, 2) + pow(right, 2) + pow(down, 2))

        if totalDistance < self._min_follow_distance:
            return await self._drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        else:
            return await self._maneuver_with_ned(front, right, down)

    async def _maneuver_with_ned(self, front, right, down):
        # get current position
        task = self.telemetry_position_velocity_ned
        current_pos = task.position

        # get angle of rotation
        task2 = self.telemetry_attitude_euler
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
