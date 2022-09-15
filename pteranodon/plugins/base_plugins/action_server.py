import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import Optional
from threading import Condition

from mavsdk import System, action_server

from .abstract_base_plugin import AbstractBasePlugin


class ActionSever(AbstractBasePlugin):
    """
    Provide vehicle actions (as a server) such as arming, taking off, and landing.
    """
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("action_server", system, loop, logger)
        self._arm_disarm_task = asyncio.ensure_future(self._arm_disarm(), loop=self._loop)
        self._arm_disarm_value: Optional[action_server.ArmDisarm] = None
        self._flight_mode_change_task = asyncio.ensure_future(self._flight_mode_change(), loop=self._loop)
        self._flight_mode_change_value: Optional[action_server.FlightMode] = None
        self._land_task = asyncio.ensure_future(self._land(), loop=self._loop)
        self._land_value: Optional[bool] = None
        self._reboot_task = asyncio.ensure_future(self._reboot(), loop=self._loop)
        self._reboot_value: Optional[bool] = None
        self._shutdown_task = asyncio.ensure_future(self._shutdown(), loop=self._loop)
        self._shutdown_value: Optional[bool] = None
        self._takeoff_task = asyncio.ensure_future(self._takeoff(), loop=self._loop)
        self._takeoff_value: Optional[bool] = None
        self._terminate_task = asyncio.ensure_future(self._terminate(), loop=self._loop)
        self._terminate_value: Optional[bool] = None

    async def _arm_disarm(self):
        async for x in self._system.action_server.arm_disarm():
            if x != self._arm_disarm_value:
                self._arm_disarm_value = x

    def arm_disarm(self) -> Optional[action_server.ArmDisarm]:
        """
        returns the current arm_disarm value
        :return: action_server.ArmDisarm ; the current arm_disarm value
        """
        return self._arm_disarm_value

    async def _flight_mode_change(self):
        async for x in self._system.action_server.flight_mode_change():
            if x != self._flight_mode_change_value:
                self._flight_mode_change_value = x

    def flight_mode_change(self) -> Optional[action_server.FlightMode]:
        """
        returns the current flight mode
        :return: action_server.FlightMode
        """
        return self._flight_mode_change_value

    def get_allowable_flight_modes(self) -> Optional[action_server.AllowableFlightModes]:
        self._logger.info(f"Pulling allowable flight modes")

        get_flight_modes_task = asyncio.ensure_future(
            self._system.action_server.get_allowable_flight_modes(), loop=self._loop
        )
        done_condition = Condition()
        get_flight_modes_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        try:
            x = get_flight_modes_task.result()
            self._logger.info("Successfully pulled allowable flight modes")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Could not pull allowable flight modes! Request timed out!")
            return None

    async def _land(self):
        async for x in self._system.action_server.land():
            if x != self._land_value:
                self._land_value = x

    def land(self) -> Optional[bool]:
        return self._land_value

    async def _reboot(self):
        async for x in self._system.action_server.reboot():
            if x != self._reboot_value:
                self._reboot_value = x

    def reboot(self) -> Optional[bool]:
        return self._reboot_value

    def set_allow_takeoff(self, allow_takeoff: bool):
        self._logger.info(f"setting allow_takeoff to {allow_takeoff}")
        super().submit_task(
            asyncio.ensure_future(self._system.action_server.set_allow_takeoff(allow_takeoff), loop=self._loop)
        )

    def set_allowable_flight_modes(self, flight_modes: action_server.AllowableFlightModes):
        self._logger.info(f"Setting allowable flight modes to inputted flight modes")
        super().submit_task(
            asyncio.ensure_future(self._system.action_server.set_allowable_flight_modes(flight_modes))
        )

    def set_armable(self, armable: bool, force_armable: bool):
        self._logger.info(f"setting \"is armable now?\" to {armable}, and \"is armable with force\" to {force_armable}")
        super().submit_task(
            asyncio.ensure_future(self._system.action_server.set_armable(armable, force_armable), loop=self._loop)
        )

    def set_disarmable(self, disarmable: bool, force_disarmable: bool):
        self._logger.info(
            f"setting \"Is disarmable\" to {disarmable}, and \"is disarmable with force\" to {force_disarmable}"
        )
        super().submit_task(
            asyncio.ensure_future(
                self._system.action_server.set_disarmable(disarmable, force_disarmable), loop=self._loop
            )
        )

    async def _shutdown(self):
        async for x in self._system.action_server.shutdown():
            if x != self._shutdown_value:
                self._shutdown_value = x

    def shutdown(self) -> Optional[bool]:
        return self._shutdown_value

    async def _takeoff(self):
        async for x in self._system.action_server.takeoff():
            if x != self._takeoff_value:
                self._takeoff_value = x

    def takeoff(self) -> Optional[bool]:
        return self._takeoff_value

    async def _terminate(self):
        async for x in self._system.action_server.terminate():
            if x != self._terminate_value:
                self._terminate_value = x

    def terminate(self) -> Optional[bool]:
        return self._terminate_value
