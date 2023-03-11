from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System, offboard

from .abstract_base_plugin import AbstractBasePlugin


class Offboard(AbstractBasePlugin):
    """
    Control a drone with position, velocity, attitude or motor commands.

    The module is called offboard because the commands can be sent from external sources as opposed to onboard
    control right inside the autopilot “board”.

    Client code must specify a setpoint before starting offboard mode. Mavsdk automatically sends setpoints at 20Hz
    (PX4 Offboard mode requires that setpoints are minimally sent at 2Hz).
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("offboard", system, loop, logger)
        self._is_active = False

        self._is_active = self._loop.run_until_complete(
            self._system.offboard.is_active()
        )

        self._end_init()

    def is_active(self) -> bool:
        """
        Check if off board control is active

        :return: True if the off board is active, False otherwise
        :rtype: bool
        """
        return self._is_active

    def set_acceleration_ned(self, accel_ned: offboard.AccelerationNed) -> None:
        """
        Set the acceleration in NED coordinates

        :param accel_ned: The NED coordinates describing accelerating
        :type accel_ned: offboard.AccelerationNed
        """
        self._submit_coroutine(self._system.offboard.set_acceleration_ned(accel_ned))

    def set_actuator_control(self, act_ctrl: offboard.ActuatorControl) -> None:
        """
        Set direct actuator control values to groups #0 and #1

        :param act_ctrl: Actuator control values
        :type act_ctrl: offboard.ActuatorControl
        """
        self._submit_coroutine(self._system.offboard.set_actuator_control(act_ctrl))

    def set_attitude(self, attitude: offboard.Attitude) -> None:
        """
        Set the attitude in terms of roll, pitch in degrees with thrust

        :param attitude: Attitude role, pitch and yaw with trust
        :type attitude: offboard.Attitude
        """
        self._submit_coroutine(self._system.offboard.set_attitude(attitude))

    def set_attitude_rate(self, attitude_rate: offboard.AttitudeRate) -> None:
        """
        Set the attitude in terms of roll, pitch and yaw alog with thrust

        :param attitude_rate: Attitude rate roll, pitch and yaw angular rate along with thrust
        :type attitude_rate: offboard.AttitudeRate
        """
        self._submit_coroutine(self._system.offboard.set_attitude_rate(attitude_rate))

    def set_position_global(self, pos_global: offboard.PositionGlobalYaw) -> None:
        """
        set the position in Global coordinates (latitude, longitude, altitude) and yaw

        :param pos_global: Position and yaw
        :type pos_global: offboard.PositionGlobalYaw
        """
        self._submit_coroutine(self._system.offboard.set_position_global(pos_global))

    def set_position_ned(self, pos_ned: offboard.PositionNedYaw) -> None:
        """
        Set the position in Ned coordinates and yaw

        :param pos_ned: Position and yaw
        :type pos_ned: offboard.PositionNedYaw
        """
        self._submit_coroutine(self._system.offboard.set_position_ned(pos_ned))

    def set_position_velocity_ned(
        self, pos: offboard.PositionNedYaw, vel: offboard.VelocityNedYaw
    ) -> None:
        """
        Set the position NED coordinates, with the velocity to be used as feed-forward.

        :param pos: Position and yaw
        :type pos: offboard.PositionNedYaw
        :param vel: Velocity and yaw
        :type vel: offboard.VelocityNedYaw
        """
        self._submit_coroutine(
            self._system.offboard.set_position_velocity_ned(pos, vel)
        )

    def set_velocity_body(self, vel_body: offboard.VelocityBodyYawspeed) -> None:
        """
        Set the velocity in body coordinates and yaw angular rate. Not available for fixed-wing aircraft

        :param vel_body: Velocity and yaw angular rate
        :type vel_body: offboard.VelocityBodyYawspeed
        """
        self._submit_coroutine(self._system.offboard.set_velocity_body(vel_body))

    def set_velocity_ned(self, vel_ned: offboard.VelocityNedYaw) -> None:
        """
        Set the velocity in NED coordinates and yaw. Not available for fixed-wing aircraft.

        :param vel_ned: Velocity and yaw
        :type vel_ned: offboard.VelocityNedYaw
        """
        self._submit_coroutine(self._system.offboard.set_velocity_ned(vel_ned))

    def start(self) -> None:
        """
        Start offboard control.
        """
        self._is_active = True
        self._schedule(
            self._system.offboard.set_attitude(offboard.Attitude(0.0, 0.0, 0.0, 0.0)),
            self._system.offboard.set_velocity_body(
                offboard.VelocityBodyYawspeed(0, 0, 0, 0)
            ),  # self.hold coroutine equivalent
            self._system.offboard.start(),
        )

    def stop(self) -> None:
        """
        Stop offboard control
        """
        self._is_active = False
        self._submit_coroutine(self._system.offboard.stop())

    def hold(self) -> None:
        """
        Hold until the drone is at the altitude defined by set_attitude
        """
        self.set_velocity_body(offboard.VelocityBodyYawspeed(0, 0, 0, 0))
