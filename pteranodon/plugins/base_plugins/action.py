from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, action

from .abstract_base_plugin import AbstractBasePlugin


class Action(AbstractBasePlugin):
    """
    Enable simple actions such as arming, taking off, and landing.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("action", system, loop, logger)

        self._maximum_speed: Optional[float] = None
        self._launch_altitude: Optional[float] = None
        self._takeoff_altitude: Optional[float] = None

        self._maximum_speed = self._loop.run_until_complete(
            self._system.action.get_maximum_speed()
        )
        self._launch_altitude = self._loop.run_until_complete(
            self._system.action.get_return_to_launch_altitude()
        )
        self._takeoff_altitude = self._loop.run_until_complete(
            self._system.action.get_takeoff_altitude()
        )

        self._end_init()

    def arm(self) -> None:
        """
        Send command to arm the drone.
        Arming a drone normally causes motors to spin at idle. Before arming take all safety precautions and stand clear
        of the drone

        """
        self._submit_coroutine(self._system.action.arm())

    def disarm(self) -> None:
        """
        Send command to disarm the drone.

        This will disarm a drone that considers itself landed. If flying, the drone should reject the disarm command.
        Disarming means that all motors will stop.
        """
        self._submit_coroutine(self._system.action.disarm())

    def do_orbit(
        self,
        radius_m: float,
        velocity_ms: float,
        yaw_behavior: action.OrbitYawBehavior,
        latitude_deg: float,
        longitude_deg: float,
        absolute_altitude_m: float,
    ) -> None:
        """
        Send command do orbit to the drone.

        :param radius_m: Radius of circle (in meters)
        :type radius_m:  float
        :param velocity_ms: Tangential velocity (in m/s)
        :type velocity_ms: float
        :param yaw_behavior: Yaw behavior of vehicle (ORBIT_YAW_BEHAVIOUR)
        :type yaw_behavior: mavsdk.action.OrbitYawBehavior
        :param latitude_deg: Center point latitude in degrees. NAN: use current latitude for center
        :type latitude_deg: float
        :param longitude_deg: Center point longitude in degrees. NAN: use current longitude for center
        :type longitude_deg: float
        :param absolute_altitude_m: Center point altitude in meters. NAN: use current altitude for center
        :type absolute_altitude_m: float
        """
        self._submit_coroutine(
            self._system.action.do_orbit(
                radius_m,
                velocity_ms,
                yaw_behavior,
                latitude_deg,
                longitude_deg,
                absolute_altitude_m,
            )
        )

    def get_maximum_speed(self) -> Optional[float]:
        """
        Get the vehicle maximum speed (in metres/second).
        """

        return self._maximum_speed

    def get_return_to_launch_altitude(self) -> Optional[float]:
        """
        Get the return to launch minimum return altitude (in meters).
        """

        return self._launch_altitude

    def get_takeoff_altitude(self) -> Optional[float]:
        """
        Get the takeoff altitude (in meters above ground).
        """

        return self._takeoff_altitude

    def goto_location(
        self,
        latitude_deg: float,
        longitude_deg: float,
        absolute_altitude_m: float,
        yaw: float,
    ) -> None:
        """
        Send command to move the vehicle to a specific global position.

        The latitude and longitude are given in degrees (WGS84 frame) and the altitude in meters AMSL
        (above mean sea level).

        :param latitude_deg: Latitude (in degrees)
        :type latitude_deg: float
        :param longitude_deg: Longitude (in degrees)
        :type longitude_deg: float
        :param absolute_altitude_m: Altitude AMSL (in meters)
        :type absolute_altitude_m: float
        :param yaw: Yaw angle (in degrees, frame is NED, 0 is North, positive is clockwise)
        :type yaw: float
        """
        self._submit_coroutine(
            self._system.action.goto_location(
                latitude_deg, longitude_deg, absolute_altitude_m, yaw
            )
        )

    def hold(self) -> None:
        """
        Send command to hold position (a.k.a. “Loiter”).

        Sends a command to drone to change to Hold flight mode, causing the vehicle to stop and maintain its current GPS
        position and altitude.

        Note: this command is specific to the PX4 Autopilot flight stack as it implies a change to a PX4-specific mode.
        """

        self._submit_coroutine(self._system.action.hold())

    def kill(self) -> None:
        """
        Send command to kill the drone.

        This will disarm a drone irrespective of whether it is landed or flying. Note that the drone will fall out of the sky
        if this command is used while flying.
        """

        self._submit_coroutine(self._system.action.kill())

    def land(self) -> None:
        """
        Send command to land at the current position.

        This switches the drone to ‘Land’ flight mode.
        """

        self._submit_coroutine(self._system.action.land())

    def reboot(self) -> None:
        """
        Send command to reboot the drone components.

        This will reboot the autopilot, companion computer, camera and gimbal.
        """

        self._submit_coroutine(self._system.action.reboot())

    def return_to_launch(self) -> None:
        """
        Send command to return to the launch (takeoff) position and land.

        This switches the drone into [Return mode](https://docs.px4.io/master/en/flight_modes/return.html) which generally means
        it will rise up to a certain altitude to clear any obstacles before heading back to the launch (takeoff) position and land there.
        """

        self._submit_coroutine(self._system.action.return_to_launch())

    def set_actuator(self, index: int, value: float) -> None:
        """
        Send command to set the value of an actuator.

        :param index: Index of the actuator
        :type index: int
        :param value: Value to set
        :type value: value
        """
        self._submit_coroutine(self._system.action.set_actuator(index, value))

    def set_current_speed(self, speed_m_s: float) -> None:
        """
        Set current speed.

        This will set the speed during a mission, reposition, and similar. It is ephemeral, so not stored on the drone
        and does not survive a reboot.

        :param speed_m_s: The speed to set in meters per second
        :type speed_m_s: float
        """
        self._submit_coroutine(self._system.action.set_current_speed(speed_m_s))

    def set_maximum_speed(self, speed_m_s: float) -> None:
        """
        Set vehicle maximum speed (in metres/second).

        :param speed_m_s: The maximum speed in meters per second
        :type speed_m_s: float
        """

        self._submit_coroutine(self._system.action.set_maximum_speed(speed_m_s))
        self._maximum_speed = speed_m_s

    def set_return_to_launch_altitude(self, relative_altitude_m: float) -> None:
        """
        Set the return to launch minimum return altitude (in meters).

        :param relative_altitude_m: The relative return to launch altitude in meters
        :type relative_altitude_m: float
        """

        self._submit_coroutine(
            self._system.action.set_return_to_launch_altitude(relative_altitude_m)
        )
        self._launch_altitude = relative_altitude_m

    def set_takeoff_altitude(self, relative_altitude_m: float) -> None:
        """
        Set takeoff altitude (in meters above ground).

        :param relative_altitude_m: The relative takeoff altitude in meters
        :type relative_altitude_m: float
        """

        self._submit_coroutine(
            self._system.action.set_takeoff_altitude(relative_altitude_m)
        )
        self._takeoff_altitude = relative_altitude_m

    def shutdown(self) -> None:
        """
        Send command to shut down the drone components.

        This will shut down the autopilot, onboard computer, camera and gimbal. This command should only be used when
        the autopilot is disarmed and autopilots commonly reject it if they are not already ready to shut down.
        """

        self._submit_coroutine(self._system.action.shutdown())

    def takeoff(self) -> None:
        """
        Send command to take off and hover.

        This switches the drone into position control mode and commands it to take off and hover at the takeoff
        altitude.

        Note that the vehicle must be armed before it can take off.
        """

        self._submit_coroutine(self._system.action.takeoff())

    def terminate(self) -> None:
        """
        Send command to terminate the drone.

        This will run the terminate routine as configured on the drone (e.g. disarm and open the parachute).
        """

        self._submit_coroutine(self._system.action.terminate())

    def transition_to_fixedwing(self) -> None:
        """
        Send command to transition the drone to fixedwing.

        The associated action will only be executed for VTOL vehicles (on other vehicle types the command will fail).
        The command will succeed if called when the vehicle is already in fixedwing mode.
        """

        self._submit_coroutine(self._system.action.transition_to_fixedwing())

    def transition_to_multicopter(self) -> None:
        """
        Send command to transition the drone to multicopter.

        The associated action will only be executed for VTOL vehicles (on other vehicle types the command will fail).
        The command will succeed if called when the vehicle is already in multicopter mode.
        """

        self._submit_coroutine(self._system.action.transition_to_multicopter())
