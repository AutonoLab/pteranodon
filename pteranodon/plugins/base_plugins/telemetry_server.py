import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from .abstract_base_plugin import AbstractBasePlugin

from mavsdk.telemetry_server import Battery, VtolState, LandedState, GroundTruth, Position, Imu, Odometry, VelocityNed, Heading


class TelemetryServer(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("telemetry_server", system, loop, logger)

    def publish_battery(self, battery: Battery) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_battery(battery), loop=self._loop)
        )

    def publish_extended_sys_state(self, vtol_state: VtolState, landed_state: LandedState) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_extended_sys_state(vtol_state, landed_state),
                                  loop=self._loop)
        )

    def publish_ground_truth(self, ground_truth: GroundTruth) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_ground_truth(ground_truth), loop=self._loop)
        )

    def publish_home(self, home: Position) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_home(home), loop=self._loop)
        )

    def publish_imu(self, imu: Imu) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_imu(imu), loop=self._loop)
        )

    def publish_odometry(self, odometry: Odometry) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_odometry(odometry), loop=self._loop)
        )

    def publish_position(self, position: Position, velocity_ned: VelocityNed, heading: Heading) -> None:
        super().submit_task(
            asyncio.ensure_future(self._system.telemetry_server.publish_position(position, velocity_ned, heading),
                                  loop=self._loop)
        )
