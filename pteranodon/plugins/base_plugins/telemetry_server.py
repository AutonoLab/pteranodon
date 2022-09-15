import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from mavsdk.telemetry_server import Battery, VtolState, LandedState, GroundTruth, Position, Imu, Odometry, VelocityNed, \
    Heading, PositionVelocityNed, RawGps, GpsInfo, StatusText

from .abstract_base_plugin import AbstractBasePlugin


class TelemetryServer(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("telemetry_server", system, loop, logger)

    def publish_battery(self, battery: Battery) -> None:
        """
        Publish to battery updates
        Args:
            battery: The next battery state
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_battery(battery), loop=self._loop)
        )

    def publish_extended_sys_state(self, vtol_state: VtolState, landed_state: LandedState) -> None:
        """
        Publish 'extended sys state' updates
        Args:
            vtol_state:
            landed_state:
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_extended_sys_state(vtol_state, landed_state),
                                  loop=self._loop)
        )

    def publish_ground_truth(self, ground_truth: GroundTruth) -> None:
        """
        Publish to 'ground truth' updates
        Args:
            ground_truth: Ground truth position information available in simulation
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_ground_truth(ground_truth), loop=self._loop)
        )

    def publish_home(self, home: Position) -> None:
        """
        Publish to 'home position' updates
        Args:
            home: The next home position
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_home(home), loop=self._loop)
        )

    def publish_imu(self, imu: Imu) -> None:
        """
        Publish to ‘IMU’ updates (in SI units in NED body frame).
        Args:
            imu: The next IMU status
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_imu(imu), loop=self._loop)
        )

    def publish_odometry(self, odometry: Odometry) -> None:
        """
        Publish to ‘odometry’ updates.
        Args:
            odometry: The next odometry status
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_odometry(odometry), loop=self._loop)
        )

    def publish_position(self, position: Position, velocity_ned: VelocityNed, heading: Heading) -> None:
        """
        Publish to ‘position’ updates.
        Args:
            position: The next position
            velocity_ned: The next velocity (NED)
            heading: Heading (yaw) in degrees
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_position(position, velocity_ned, heading),
                                  loop=self._loop)
        )

    def publish_position_velocity_ned(self, position_velocity_ned: PositionVelocityNed) -> None:
        """
        Publish to ‘position velocity’ updates.
        Args:
            position_velocity_ned: The next position and velocity status
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_position_velocity_ned(position_velocity_ned),
                                  loop=self._loop)
        )

    def publish_raw_gps(self, raw_gps: RawGps, gps_info: GpsInfo) -> None:
        """
        Publish to ‘Raw GPS’ updates.
        Args:
            raw_gps: The next ‘Raw GPS’ state. Warning: this is an advanced feature, use Position updates to get the
            location of the drone!
            gps_info: The next ‘GPS info’ state
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_raw_gps(raw_gps, gps_info), loop=self._loop)
        )

    def publish_raw_imu(self, imu: Imu) -> None:
        """
        Publish to ‘Raw IMU’ updates.
        Args:
            imu: The next raw IMU status
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_raw_imu(imu), loop=self._loop)
        )

    def publish_scaled_imu(self, imu: Imu) -> None:
        """
        Publish to ‘Scaled IMU’ updates.
        Args:
            imu: The next scaled IMU status
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_scaled_imu(imu), loop=self._loop)
        )

    def publish_status_text(self, status_text: StatusText) -> None:
        """
        Publish to ‘status text’ updates.
        Args:
            status_text:  The next ‘status text’
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_status_text(status_text), loop=self._loop)
        )

    def publish_sys_status(self, battery: Battery, rc_receiver_status: bool, gyro_status: bool, accel_status: bool,
                           mag_status: bool, gps_status: bool) -> None:
        """
        Publish ‘sys status’ updates.
        Args:
            battery: The next ‘battery’ state
            rc_receiver_status: rc receiver status
            mag_status:
            gps_status:
            accel_status:
            gyro_status:
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_sys_status(battery, rc_receiver_status,
                                                                                   gyro_status, accel_status,
                                                                                   mag_status, gps_status),
                                  loop=self._loop)
        )

    # not sure if this is int or u_int64 on line 88 as param
    def publish_unix_epoch_time(self, time_us: int) -> None:
        """
        Publish to ‘unix epoch time’ updates.
        Args:
            time_us: The next ‘unix epoch time’ status
        """

        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_unix_epoch_time(time_us), loop=self._loop)
        )
