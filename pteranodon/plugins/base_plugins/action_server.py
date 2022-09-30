import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional
from threading import Condition

from mavsdk import System, action_server

from .abstract_base_plugin import AbstractBasePlugin


class ActionServer(AbstractBasePlugin):
    """
    Provide vehicle actions (as a server) such as arming, taking off, and landing.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("action_server", system, loop, logger)
        self._arm_disarm_task = asyncio.run_coroutine_threadsafe(
            self._arm_disarm(), loop=self._loop
        )
        self._arm_disarm_value: Optional[action_server.ArmDisarm] = None
        self._flight_mode_change_task = asyncio.run_coroutine_threadsafe(
            self._flight_mode_change(), loop=self._loop
        )
        self._flight_mode_change_value: Optional[action_server.FlightMode] = None
        self._land_task = asyncio.run_coroutine_threadsafe(
            self._land(), loop=self._loop
        )
        self._land_value: Optional[bool] = None
        self._reboot_task = asyncio.run_coroutine_threadsafe(
            self._reboot(), loop=self._loop
        )
        self._reboot_value: Optional[bool] = None
        self._shutdown_task = asyncio.run_coroutine_threadsafe(
            self._shutdown(), loop=self._loop
        )
        self._shutdown_value: Optional[bool] = None
        self._takeoff_task = asyncio.run_coroutine_threadsafe(
            self._takeoff(), loop=self._loop
        )
        self._takeoff_value: Optional[bool] = None
        self._terminate_task = asyncio.run_coroutine_threadsafe(
            self._terminate(), loop=self._loop
        )
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

    def get_allowable_flight_modes(
        self,
    ) -> Optional[action_server.AllowableFlightModes]:
        """
        Returns the flight modes allowed
        :return: action_server.AllowableFlightModes ; all allowed flight modes
        """
        self._logger.info("Pulling allowable flight modes")

        get_flight_modes_task = asyncio.run_coroutine_threadsafe(
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
            self._logger.error(
                "Could not pull allowable flight modes! Request timed out!"
            )
            return None

    async def _land(self):
        async for x in self._system.action_server.land():
            if x != self._land_value:
                self._land_value = x

    def land(self) -> Optional[bool]:
        """
        returns the boolean that explains if the vehicle is landing
        :return: bool ; True if the vehicle is landing, False otherwise
        """
        return self._land_value

    async def _reboot(self):
        async for x in self._system.action_server.reboot():
            if x != self._reboot_value:
                self._reboot_value = x

    def reboot(self) -> Optional[bool]:
        """
        returnsa the boolean that explains if the vehicle is rebooting
        :return: bool ; True if the vehicle is rebooting, False otherwise
        """
        return self._reboot_value

    def set_allow_takeoff(self, allow_takeoff: bool):
        """
        Sets the boolean that allows the vehicle to takeoff
        :param allow_takeoff: bool ; True if the vehicle is allowed to take off, False otherwise
        :return: None
        """
        self._logger.info(f"setting allow_takeoff to {allow_takeoff}")
        self._submit_coroutine(
            self._system.action_server.set_allow_takeoff(allow_takeoff)
        )

    def set_allowable_flight_modes(
        self, flight_modes: action_server.AllowableFlightModes
    ):
        """
        Sets the flight modes allowable by the vehicle
        :param flight_modes: action_server.AllowableFlightModes ; a list of allowable flight modes
        :return: None
        """
        self._logger.info("Setting allowable flight modes to inputted flight modes")
        self._submit_coroutine(
            self._system.action_server.set_allowable_flight_modes(flight_modes)
        )

    def set_armable(self, armable: bool, force_armable: bool):
        """
        Sets if the vehicle is armable or force_armable
        :param armable: bool ; True if the vehicle is armable, False otherwise
        :param force_armable: bool ; True if the vehicle is force_armable, False otherwise
        :return: None
        """
        self._logger.info(
            f'setting "is armable now?" to {armable}, and "is armable with force" to {force_armable}'
        )
        self._submit_coroutine(
            self._system.action_server.set_armable(armable, force_armable)
        )

    def set_disarmable(self, disarmable: bool, force_disarmable: bool):
        """
        Sets if the vehicle is disarmable or force_disarmable
        :param disarmable: bool ; True if the vehicle is disarmable, False otherwise
        :param force_disarmable: bool ; True if the vehicle is force_disarmable, False otherwise
        :return: None
        """
        self._logger.info(
            f'setting "Is disarmable" to {disarmable}, and "is disarmable with force" to {force_disarmable}'
        )
        self._submit_coroutine(
            self._system.action_server.set_disarmable(disarmable, force_disarmable)
        )

    async def _shutdown(self):
        async for x in self._system.action_server.shutdown():
            if x != self._shutdown_value:
                self._shutdown_value = x

    def shutdown(self) -> Optional[bool]:
        """
        returns if the vehicle is in the process of shutting down
        :return: bool ; True if the vehicle is shutting down, False otherwise
        """
        return self._shutdown_value

    async def _takeoff(self):
        async for x in self._system.action_server.takeoff():
            if x != self._takeoff_value:
                self._takeoff_value = x

    def takeoff(self) -> Optional[bool]:
        """
        Returns if the vehicle is in the state of taking off
        :return: bool ; True if the vehicle is taking off, false otherwise
        """
        return self._takeoff_value

    async def _terminate(self):
        async for x in self._system.action_server.terminate():
            if x != self._terminate_value:
                self._terminate_value = x

    def terminate(self) -> Optional[bool]:
        """
        Returns if the mission is being terminated
        :return: bool ; True if the mission is terminated, False othewise
        """
        return self._terminate_value
