from threading import Thread
from time import sleep, perf_counter
from collections import deque
import asyncio
from concurrent.futures import Future
import atexit
from abc import abstractmethod, ABC
from typing import Any, List, Tuple, Callable, Dict, Optional, Union
import logging
import sys
import random
import signal

from mavsdk import System
from mavsdk.offboard import OffboardError
from mavsdk.action import ActionError


try:
    import uvloop

    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
except ModuleNotFoundError:
    pass

from .server_detector import ServerDetector

from .plugins import PluginManager
from .plugins.base_plugins import (
    ActionServer,
    Action,
    Calibration,
    CameraServer,
    Camera,
    ComponentInformationServer,
    ComponentInformation,
    Core,
    Failure,
    FollowMe,
    Ftp,
    Geofence,
    Gimbal,
    Info,
    LogFiles,
    ManualControl,
    MissionRawServer,
    MissionRaw,
    Mission,
    Mocap,
    Offboard,
    ParamServer,
    Param,
    Rtk,
    ServerUtility,
    Shell,
    TelemetryServer,
    Telemetry,
    TrackingServer,
    Transponder,
    Tune,
)
from .plugins.extension_plugins import Sensor, Relative


class AbstractDrone(ABC):
    """
    An abstract class with base functionality, some methods must be overridden for usage
    """

    def __init__(
        self,
        address: str,
        log_file_name: Optional[str] = None,
        time_slice=0.050,
        **kwargs,
    ):
        """
        :param address: Connection address for use with mavsdk.System.connect method
        :param time_slice: The interval to process commands in the queue
        :param min_follow_distance: The minimum distance a point must be from the drone, for a movement to take place
        in the maneuver_to method
        """
        # attatch signal handlers
        self._handle_signals_main()

        # setup the logger first
        logger_name = "mavlog.log" if log_file_name is None else log_file_name
        self._logger = ServerDetector.setup_logger(logger_name)

        # set up the instance fields
        self._stopped_mavlink = False
        self._stopped_loop = False
        self._time_slice = time_slice
        self._address = address

        # setup resources for drone control, mavsdk.System, deque, thread, etc..
        self._drone = System(port=random.randint(1000, 65535))
        self._queue: deque = deque()
        self._task_cache: deque = deque(maxlen=10)
        self._loop = asyncio.get_event_loop()
        self._mavlink_thread = Thread(
            name="Mavlink-Command-Thread",
            target=self._process_command_loop,
            daemon=True,
        )

        # make stuff for telemetry
        self._telemetry_thread = Thread(
            name="Mavlink-Telemetry-Thread",
            target=self._run_telemetry_loop,
            daemon=True,
        )

        # register the stop method so everything cleans up
        self._ran_stop = False
        atexit.register(self.stop)

        # connect the drone
        self._connect()

        try:
            # setup all plugins
            self._plugins = PluginManager(self._drone, self._loop, self._logger, kwargs)

            # after connection run setup, then initialize loop, run teardown during cleanup phase
            self.setup()
            self._loop_thread = Thread(
                name="Loop-Thread", target=self._loop_loop, daemon=True
            )

            # finally, start the mavlink thread
            self._mavlink_thread.start()
            self._telemetry_thread.start()
            self.sensor.start_all_sensors()

            # create a list of threads in the object
            self._threads: List[Thread] = [
                self._loop_thread,
                self._mavlink_thread,
                self._telemetry_thread,
            ]
        except Exception as e:
            self._cleanup()
            raise e

    # PLUGIN PROPERTIES
    @property
    def plugins(self) -> PluginManager:
        """
        The plugin manager instance
        """
        return self._plugins

    @property
    def action_server(self) -> ActionServer:
        """
        The ActionServer plugin instance
        """
        return self._plugins.action_server

    @property
    def action(self) -> Action:
        """
        The Action plugin instance
        """
        return self._plugins.action

    @property
    def calibration(self) -> Calibration:
        """
        The Calibration plugin instance
        """
        return self._plugins.calibration

    @property
    def camera_server(self) -> CameraServer:
        """
        The CameraServer plugin instance
        """
        return self._plugins.camera_server

    @property
    def camera(self) -> Camera:
        """
        The Camera plugin instance
        """
        return self._plugins.camera

    @property
    def component_information_server(self) -> ComponentInformationServer:
        """
        The ComponentInformationServer plugin instance
        """
        return self._plugins.component_information_server

    @property
    def component_information(self) -> ComponentInformation:
        """
        The ComponentInformation plugin instance
        """
        return self._plugins.component_information

    @property
    def core(self) -> Core:
        """
        The Core plugin instance
        """
        return self._plugins.core

    @property
    def failure(self) -> Failure:
        """
        The Failure plugin instance
        """
        return self._plugins.failure

    @property
    def follow_me(self) -> FollowMe:
        """
        The FollowMe plugin instance
        """
        return self._plugins.follow_me

    @property
    def ftp(self) -> Ftp:
        """
        The Ftp plugin instance
        """
        return self._plugins.ftp

    @property
    def geofence(self) -> Geofence:
        """
        The Geofence plugin instance
        """
        return self._plugins.geofence

    @property
    def gimbal(self) -> Gimbal:
        """
        The  plugin instance
        """
        return self._plugins.gimbal

    @property
    def info(self) -> Info:
        """
        The Info plugin instance
        """
        return self._plugins.info

    @property
    def log_files(self) -> LogFiles:
        """
        The LogFiles plugin instance
        """
        return self._plugins.log_files

    @property
    def manual_control(self) -> ManualControl:
        """
        The ManualControl plugin instance
        """
        return self._plugins.manual_control

    @property
    def mission_raw_server(self) -> MissionRawServer:
        """
        The MissionRawServer plugin instance
        """
        return self._plugins.mission_raw_server

    @property
    def mission_raw(self) -> MissionRaw:
        """
        The MissionRaw plugin instance
        """
        return self._plugins.mission_raw

    @property
    def mission(self) -> Mission:
        """
        The Mission plugin instance
        """
        return self._plugins.mission

    @property
    def mocap(self) -> Mocap:
        """
        The Mocap plugin instance
        """
        return self._plugins.mocap

    @property
    def offboard(self) -> Offboard:
        """
        The Offboard plugin instance
        """
        return self._plugins.offboard

    @property
    def param_server(self) -> ParamServer:
        """
        The ParamServer plugin instance
        """
        return self._plugins.param_server

    @property
    def param(self) -> Param:
        """
        The Param plugin instance
        """
        return self._plugins.param

    @property
    def rtk(self) -> Rtk:
        """
        The Rtk plugin instance
        """
        return self._plugins.rtk

    @property
    def server_utility(self) -> ServerUtility:
        """
        The ServerUtility plugin instance
        """
        return self._plugins.server_utility

    @property
    def shell(self) -> Shell:
        """
        The Shell plugin instance
        """
        return self._plugins.shell

    @property
    def telemetry_server(self) -> TelemetryServer:
        """
        The TelemetryServer plugin instance
        """
        return self._plugins.telemetry_server

    @property
    def telemetry(self) -> Telemetry:
        """
        The Telemetry plugin instance
        """
        return self._plugins.telemetry

    @property
    def tracking_server(self) -> TrackingServer:
        """
        The TrackingServer plugin instance
        """
        return self._plugins.tracking_server

    @property
    def transponder(self) -> Transponder:
        """
        The Transponder plugin instance
        """
        return self._plugins.transponder

    @property
    def tune(self) -> Tune:
        """
        The Tune plugin instance
        """
        return self._plugins.tune

    @property
    def sensor(self) -> Sensor:
        """
        :return: The Sensor plugin class instance
        """
        return self._plugins.sensor

    @property
    def relative(self) -> Relative:
        """
        :return: The Relative plugin class instance
        """
        return self._plugins.relative

    # NON-PLUGIN PROPERTIES
    @property
    def system(self) -> System:
        """
        :return: The MAVSDK System instance
        """
        return self._drone

    @property
    def logger(self) -> logging.Logger:
        """
        :return: The logger instance setup in the __init__ method of AbstractDrone
        """
        return self._logger

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

    async def _teardown(self) -> None:
        for command, handler, args, kwargs in self._queue:
            self._process_command(command, handler, args, kwargs)

    ###############################################################################
    # internal methods for drone control, connection, threading of actions, etc..
    ###############################################################################
    def _connect(self) -> None:
        try:
            self._loop.run_until_complete(
                self._drone.connect(system_address=self.address)
            )
        except KeyboardInterrupt as e:
            self._cleanup()
            raise KeyboardInterrupt from e
        finally:
            self._loop.run_until_complete(self._async_connect())

    async def _async_connect(self):
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                self._logger.info("Drone connection established")
                break

    def _cleanup(self) -> None:
        try:
            self._drone.__del__()  # mypy: ignore # pylint: disable=unnecessary-dunder-call
            del self._drone
        except AttributeError:
            pass

    def _force_cleanup(self) -> None:
        self._cleanup()
        sys.exit(-1)

    def _handle_signals_main(self) -> None:
        signal.signal(signal.SIGTERM, lambda a, b: self._force_cleanup)
        signal.signal(signal.SIGINT, lambda a, b: self._force_cleanup)

    def _process_command(
        self, com: Callable, handler: Optional[Callable], args: List, kwargs: Dict
    ) -> None:

        if com is None:
            return

        self._logger.info(
            f"Processing: {com.__module__}.{com.__qualname__} with args: {args} and kwargs: {kwargs}"
        )
        if asyncio.iscoroutinefunction(com):  # if it is an async function
            new_future: Future = asyncio.run_coroutine_threadsafe(
                com(*args, **kwargs), loop=self._loop
            )
            new_future.add_done_callback(
                lambda f: self._logger.info(
                    f"Completed: {f.__module__}.{f.__qualname__} with args: {args} and kwargs: {kwargs}"  # type: ignore
                )
            )
            self._task_cache.append(new_future)
        else:  # typical sync function
            com(*args, **kwargs)

        # If a handler is present, call it when the command has been processed
        if handler is not None:
            handler()

    # loop for processing mavlink commands. Uses the time slice determined in the contructor for its maximum rate.
    def _process_command_loop(self) -> None:
        try:
            while not self._stopped_mavlink:
                start_time = perf_counter()
                try:
                    com, handler, args, kwargs = self._queue.popleft()
                    if args is None:
                        args = []
                    if kwargs is None:
                        kwargs = {}
                    self._process_command(com, handler, args, kwargs)
                # TODO, perform logging on these exceptions
                except IndexError as e:
                    # this exception is expected since we expect the deque to not have elements sometimes
                    if str(e) != "pop from an empty deque":
                        # if the exception makes it here, it is unexpected
                        self._logger.error(e)
                except ActionError as e:
                    self._logger.error(e)
                except OffboardError as e:
                    self._logger.error(e)
                finally:
                    end_time = perf_counter() - start_time
                    if end_time < self._time_slice:
                        sleep(self._time_slice - end_time)
        finally:
            self._cleanup()
            pass

    # method for running telemetry data
    def _run_telemetry_loop(self):
        self._loop.run_forever()

    # method which joins a thread with a timeout, used with map to close all threads\
    @staticmethod
    def _join_thread(thread: Thread, timeout=1) -> None:
        try:
            thread.join(timeout=timeout)
        except RuntimeError:
            pass

    # method to stop the thread for processing mavlink commands
    def stop(self) -> None:
        """
        Clears the command queue, attempts to disarm the drone, closes all telemetry readings, stops the loop method
        execution, stops the mavlink command loop, and then joins the mavlink thread.
        :return: None
        """
        if self._ran_stop:
            return
        self._ran_stop = True

        # shutdown any generators (yield) which are running
        # don't save the Future since cleaning and shutting down anyways
        self._logger.info("Closing async generators")
        self._plugins.cancel_all_futures()
        asyncio.run_coroutine_threadsafe(
            self._loop.shutdown_asyncgens(), loop=self._loop
        )  # shutdown_asyncgens is a coroutine

        # on a stop call put a disarm call in the empty queue
        self._queue.clear()

        # Likely not needed
        # self.put(self.action.disarm)

        # shutdown the any asyncgens that have been opened
        self._stopped_mavlink = True
        self._stopped_loop = True
        self._loop.stop()

        # attempt to join all threads
        self._logger.info("Attempting to join threads")
        map(self._join_thread, self._threads)

        # run teardown (queue should be clean from the clear before disarm)
        self._logger.info("Running remaining async Tasks/Future, and MAVSDK commands")
        asyncio.get_event_loop().run_until_complete(self._teardown())

        # cleanup the mavsdk.System instance
        self._cleanup()

        # close logging
        ServerDetector.close_logger(self._logger)

    # method for queueing mavlink commands
    # TODO, implement a clear queue or priority flag. using deque allows these operations to be deterministic

    # Can be either a function, or a function and a handler
    def put(
        self, obj: Union[Callable, Tuple[Callable, Callable]], *args: Any, **kwargs: Any
    ) -> None:
        """
        Used to put functions/methods in a queue for execution in a separate thread asynchronously or otherwise
        :param obj: A callable function/method which will get executed in the mavlink command loop thread
        :param args: The arguments for the function/method
        :param kwargs: The keyword arguments for the function/method
        :return: None
        """

        command: Callable
        handler: Optional[Callable] = None
        if isinstance(obj, tuple):
            command, handler = obj
        else:
            command = obj

        self._logger.info(
            f"User called: {command.__module__}.{command.__qualname__}, putting call in queue"
        )
        self._queue.append((command, handler, args, kwargs))

    ##################################################
    # TODO, implement the flag for put in these methods
    def arm(self) -> None:
        """
        Arms the drone.
        :return: None
        """
        self.put(self.action.arm)

    def disarm(self) -> None:
        """
        Disarms the drone.
        :return: None
        """
        self.put(self.action.disarm)

    def hold(self) -> None:
        """
        Send command to hold current position
        Note: this command is specific to the PX4 Autopilot flight stack as it implies a change to a PX4-specific mode.
        :return: None
        """
        self.put(self.action.hold)

    def land(self) -> None:
        """
        Send command to land the drone
        :return: None
        """
        self.put(self.action.land)

    def return_to_launch(self) -> None:
        """
        Send command to return to launch (takeoff) position and land.
        :return: None
        """
        self.put(self.action.return_to_launch)

    def set_maximum_speed(self, *args: float, **kwargs: float) -> None:
        """
        Set vehicle maximum speed (in metres/second).
        :param speed: Maximum speed in m/s
        :return:None
        """
        self.put(self.action.set_maximum_speed, *args, **kwargs)

    def set_return_to_launch_altitude(self, *args: float, **kwargs: float) -> None:
        """
        Set the return to launch minimum return altitude (in meters).
        :param relative_altitude_m: Altitude relative to takeoff location
        :return: None
        """
        self.put(self.action.set_return_to_launch_altitude, *args, **kwargs)

    def set_takeoff_altitude(self, *args: float, **kwargs: float) -> None:
        """
        Set takeoff altitude (in meters above ground).
        :param altitude: Takeoff altitude relative to ground/takeoff location
        :return: None
        """
        self.put(self.action.set_takeoff_altitude, *args, **kwargs)

    def shutdown(self) -> None:
        """
        Send command to shut down the drone components.
        :return: None
        """
        self.put(self.action.shutdown)

    def takeoff(self) -> None:
        """
        Send command to take off and hover.
        :return: None
        """
        self.put(self.action.takeoff)

    def maneuver_to(
        self,
        front: float,
        right: float,
        down: float,
        on_dimensions: Tuple = (True, True, True),
        test_min: bool = False,
    ):
        """
        A movement command for moving relative to the drones current position. The front direction is aligned directly with
        the drones front as defined in the configuration.
        :param front: Relative distance in front of drone
        :param right: Relative distance to the right of drone
        :param down: Relative distance below the drone
        :param on_dimensions: A tuple of 3 boolean values. In order they represent if the drone will move
        (front, right, down). If set to False the drone will not move in that direction
        """
        self.put(self.relative.maneuver_to, front, right, down, on_dimensions, test_min)

    def create_geofence(self, distance: float) -> None:
        """
        Creates a relative inclusive geofence around the drones home coordinates. The geofence is defined as a square
        where the distance parameter is equal to half the side length.
        :param distance: The meters from home the drone can maneuver
        :return: None
        """
        self.put(self.relative.create_geofence, distance)
