from threading import Thread
from time import sleep, perf_counter
from collections import deque
import asyncio
import atexit
from math import atan, degrees, sqrt, pow, cos, sin, radians
from abc import abstractmethod, ABC
from typing import Union, Any, List, Tuple, Callable, Dict, Optional

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed, Attitude, OffboardError
from mavsdk.action import ActionError, OrbitYawBehavior
import mavsdk.telemetry as telemetry


class AbstractDrone(ABC):
    def __init__(self, address: str, time_slice=0.050, min_follow_distance=5.0):
        """
        :param address: Connection address for use with mavsdk.System.connect method
        :param time_slice: The interval to process commands in the queue
        :param min_follow_distance: The minimum distance a point must be from the drone, for a movement to take place
        in the maneuver_to method
        """
        # set up the instance fields
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

        # finally, start the mavlink thread
        self._mavlink_thread.start()
        sleep(0.002)  # short sleep to ensure thread is started

    # properties for the class
    @property
    def address(self) -> str:
        """
        :return: The address used to connect in mavsdk.System.connect
        """
        return self._address

    @property
    def time_slice(self) -> float:
        """
        :return: The time slice the drone is currently using
        """
        return self._time_slice

    @time_slice.setter
    def time_slice(self, val: float):
        """
        :param val: A new time slice for the drone to use
        """
        self._time_slice = val

    @property
    def min_follow_distance(self) -> float:
        """
        :return: The minimum following distance that is currently being used
        """
        return self._min_follow_distance

    @min_follow_distance.setter
    def min_follow_distance(self, dist: float):
        """
        :param dist: The new minimum distance that the drone should use in the maneuver_to method
        """
        self._min_follow_distance = dist

    # AREA TO OVERRIDE
    # abstract methods which over classes must override
    @abstractmethod
    def setup(self) -> None:
        """
        A function which takes no parameters and does not have any return value. This is implemented in the concrete
        implementations of the AbstractDrone class. This method's responsibility is to handle any one-time operations
        that need to occur. These operations could occur in the __init__, but the setup runs after a connection has
        been established to the flight controller, so methods such as start_offboard or even arm could occur here.
        Should not be called itself.
        """
        pass

    @abstractmethod
    def loop(self) -> None:
        """
        A function which takes no parameters and does not have any return value. This method is run continously once
        a call to start_loop occurs. This could handle the sending of certain sensor data to a control algorithm or
        simply print some info every so often.
        Should not be called itself.
        """
        pass

    def _loop_loop(self) -> None:
        while not self._stopped_loop:
            self.loop()

    def start_loop(self) -> None:
        """
        Begins the execution of the loop method. Starts the internal thread.
        """
        self._loop_thread.start()

    @abstractmethod
    def teardown(self) -> None:
        """
        A function which takes no parameters and does not have any return value. This method runs just before the
        mavlink thread is stopped and joined. As such it has access to calling methods such as stop_offboard and land.
        Should not be called itself.
        """
        pass

    ###############################################################################
    # internal methods for drone control, connection, threading of actions, etc..
    ###############################################################################
    def _connect(self) -> None:
        try:
            self._loop.run_until_complete(self._drone.connect(system_address=self.address))
        except KeyboardInterrupt:
            self._cleanup()
            raise KeyboardInterrupt
        finally:
            pass  # todo, something?

    def _cleanup(self) -> None:
        self._drone.__del__()
        del self._drone

    def _process_command(self, com: Callable, args: List, kwargs: Dict) -> None:
        # print(f"Printing in process_com:\n{args}\n{kwargs}\nDone printing in process_com")
        if com is None:
            pass  # just means it timed out without a new command
        elif asyncio.iscoroutinefunction(com):  # if it is an async function
            _ = self._loop.run_until_complete(com(*args, **kwargs))
        else:  # typical sync function
            _ = com(*args, **kwargs)

    # loop for processing mavlink commands. Uses the time slice determined in the contructor for its maximum rate. 
    def _process_command_loop(self) -> None:
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
    def stop(self) -> None:
        """
        Clears the command queue, attempts to disarm the drone, closes all telemetry readings, stops the loop method
        execution, stops the mavlink command loop, and then joins the mavlink thread.
        :return: None
        """
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
    def put(self, obj: Callable, *args: Any, **kwargs: Any) -> None:
        """
        Used to put functions/methods in a queue for execution in a separate thread asynchronously or otherwise
        :param obj: A callable function/method which will get executed in the mavlink command loop thread
        :param args: The arguments for the function/method
        :param kwargs: The keyword arguments for the function/method
        :return: None
        """
        # print(f"Printing in put:\n{args}\n{kwargs}\nDone printing in put")
        self._queue.append((obj, args, kwargs))

    ##################################################
    # methods which implement the mavsdk System actions
    ##################################################
    # TODO, implement the flag for put in these methods
    def arm(self) -> None:
        """
        Arms the drone.
        :return: None
        """
        self.put(self._drone.action.arm)

    def disarm(self) -> None:
        """
        Disarms the drone.
        :return: None
        """
        self.put(self._drone.action.disarm)

    def do_orbit(self, *args: Union[float, OrbitYawBehavior], **kwargs: Union[float, OrbitYawBehavior]) -> None:
        """
        Performs an orbit in midair
        :param radius_m: Radius of circle (in meters)
        :param velocity_ms: Tangential velocity (in m/s)
        :param yaw_behavior: An OrbitYawBehavior
        :param latitude_deg: Optional: Center point in latitude in degrees, uses current point as default
        :param longitude_deg: Optional: Center point in longitude in degrees, uses current point as default
        :param absolute_altitude_m: Optional: Center point altitude in meters, uses current point as default
        :return: None
        """
        self.put(self._drone.action.do_orbit, *args, **kwargs)

    def get_maximum_speed(self) -> float:
        """
        Get the vehicle maximum speed (in metres/second).
        :return: Maximum speed (in metres/second)
        """
        return self._loop.run_until_complete(self._drone.action.get_maximum_speed())

    def get_return_to_launch_altitude(self) -> float:
        """
        Get the return to launch minimum return altitude (in meters).
        :return: Return altitude relative to takeoff location (in meters)
        """
        return self._loop.run_until_complete(self._drone.action.get_return_to_launch_altitude())

    def get_takeoff_altitude(self) -> float:
        """
        Get the takeoff altitude (in meters above ground).
        :return: Takeoff altitude relative to ground/takeoff location (in meters)
        """
        return self._loop.run_until_complete(self._drone.action.get_takeoff_altitude())

    def goto_location(self, *args: float, **kwargs: float) -> None:
        """
        Send command to move the vehicle to a specific global position.
        The latitude and longitude are given in degrees (WGS84 frame) and the altitude in meters AMSL (above mean sea
        level).
        The yaw angle is in degrees (frame is NED, 0 is North, positive is clockwise).
        :param latitude_deg: Latitude (in degrees)
        :param longitude_deg: Longitude (in degrees)
        :param absolute_altitude_m: Altitude AMSL (in meters)
        :param yaw_deg: Yaw angle in degrees (0 is North, positive is clockwise)
        :return: None
        """
        self.put(self._drone.action.goto_location, *args, **kwargs)

    def hold(self) -> None:
        """
        Send command to hold current position
        Note: this command is specific to the PX4 Autopilot flight stack as it implies a change to a PX4-specific mode.
        :return: None
        """
        self.put(self._drone.action.hold)

    def kill(self) -> None:
        """
        Send command to kill the drone.
        :return: None
        """
        self.put(self._drone.action.kill)

    def land(self) -> None:
        """
        Send command to land the drone
        :return: None
        """
        self.put(self._drone.action.land)

    def reboot(self) -> None:
        """
        Send command to reboot drone components
        :return: None
        """
        self.put(self._drone.action.reboot)

    def return_to_launch(self) -> None:
        """
        Send command to return to launch (takeoff) position and land.
        :return: None
        """
        self.put(self._drone.action.return_to_launch)

    def set_actuator(self, *args: Union[int, float], **kwargs: Union[int, float]) -> None:
        """
        Uses a value to set an acuator
        :param index: Index of actuator (starting at 1)
        :param value: Value to set the acuator to (normalized from [-1..1])
        :return: None
        """
        self.put(self._drone.action.set_actuator, *args, **kwargs)

    def set_current_speed(self, *args: float, **kwargs: float) -> None:
        """
        Set current speed
        :param speed_m_s: Speed in m/s
        :return: None
        """
        self.put(self._drone.action.set_current_speed, *args, **kwargs)

    def set_maximum_speed(self, *args: float, **kwargs: float) -> None:
        """
        Set vehicle maximum speed (in metres/second).
        :param speed: Maximum speed in m/s
        :return:None
        """
        self.put(self._drone.action.set_maximum_speed, *args, **kwargs)

    def set_return_to_launch_altitude(self, *args: float, **kwargs: float) -> None:
        """
        Set the return to launch minimum return altitude (in meters).
        :param relative_altitude_m: Altitude relative to takeoff location
        :return: None
        """
        self.put(self._drone.action.set_return_to_launch_altitude, *args, **kwargs)

    def set_takeoff_altitude(self, *args: float, **kwargs: float) -> None:
        """
        Set takeoff altitude (in meters above ground).
        :param altitude: Takeoff altitude relative to ground/takeoff location
        :return: None
        """
        self.put(self._drone.action.set_takeoff_altitude, *args, **kwargs)

    def shutdown(self) -> None:
        """
        Send command to shut down the drone components.
        :return: None
        """
        self.put(self._drone.action.shutdown)

    def takeoff(self) -> None:
        """
        Send command to take off and hover.
        :return: None
        """
        self.put(self._drone.action.takeoff)

    def terminate(self) -> None:
        """
        Send command to terminate the drone.
        :return: None
        """
        self.put(self._drone.action.terminate)

    def transition_to_fixedwing(self) -> None:
        """
        Send command to transition the drone to fixedwing.
        :return: None
        """
        self.put(self._drone.action.transition_to_fixedwing)

    def transition_to_multicopter(self) -> None:
        """
        Send command to transition the drone to multicopter.
        :return: None
        """
        self.put(self._drone.action.transition_to_multicopter)

    #########################################################
    # methods for the mavsdk System telemetry 
    #########################################################
    def _get_telemetry(self, prop):
        pass  # TODO, idea is to have this be a wrapper to check if a telemetry is null and execute some behavior

    async def _get_gps_info(self) -> None:
        async for gps_info in self._drone.telemetry.gps_info():
            if gps_info != self._telemetry_gps_info:
                self._telemetry_gps_info = gps_info

    @property
    def telemetry_gps_info(self) -> telemetry.GpsInfo:
        return self._telemetry_gps_info

    async def _get_position_velocity_ned(self):
        async for position in self._drone.telemetry.position_velocity_ned():
            if position != self._telemetry_position_velocity_ned:
                self._telemetry_position_velocity_ned = position

    @property
    def telemetry_position_velocity_ned(self) -> telemetry.PositionVelocityNed:
        return self._telemetry_position_velocity_ned

    async def _get_battery(self):
        async for battery in self._drone.telemetry.battery():
            if battery != self._telemetry_battery:
                self._telemetry_battery = battery

    @property
    def telemetry_battery(self) -> telemetry.Battery:
        return self._telemetry_battery

    async def _get_attitude_euler(self):
        async for attitude in self._drone.telemetry.attitude_euler():
            if attitude != self._telemetry_attitude_euler:
                self._telemetry_attitude_euler = attitude

    @property
    def telemetry_attitude_euler(self) -> telemetry.EulerAngle:
        return self._telemetry_attitude_euler

    #########################################################
    # methods for using offboard
    #########################################################
    def start_offboard(self) -> None:
        self.put(self._drone.offboard.set_attitude, Attitude(0.0, 0.0, 0.0, 0.0))
        self.offboard_hold()
        self.put(self._drone.offboard.start)

    def stop_offboard(self) -> None:
        self.offboard_hold()
        self.put(self._drone.offboard.stop)

    def offboard_hold(self) -> None:
        self.put(self._drone.offboard.set_velocity_body, VelocityBodyYawspeed(0, 0, 0, 0))

    def offboard_set_velocity_body(self, *args: VelocityBodyYawspeed, **kwargs: VelocityBodyYawspeed) -> None:
        self.put(self._drone.offboard.set_velocity_body, *args, **kwargs)

    def offboard_set_position_ned(self, *args: PositionNedYaw, **kwargs: PositionNedYaw) -> None:
        self.put(self._drone.offboard.set_position_ned, *args, **kwargs)

    #########################################################
    # methods for maneuvering
    #########################################################
    def maneuver_to(self, front: float, right: float, down: float):
        """
        param:: front -> float
        param:: right -> float
        param:: down -> float
        """
        self.put(self._maneuver_to, front, right, down)

    async def _maneuver_to(self, front: float, right: float, down: float) -> None:
        totalDistance = sqrt(pow(front, 2) + pow(right, 2) + pow(down, 2))

        if totalDistance < self._min_follow_distance:
            return await self._drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        else:
            return await self._maneuver_with_ned(front, right, down)

    async def _maneuver_with_ned(self, front: float, right: float, down: float) -> None:
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
