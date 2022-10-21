from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, action_server

from .abstract_base_plugin import AbstractBasePlugin


class ActionServer(AbstractBasePlugin):
    """
    Provide vehicle actions (as a server) such as arming, taking off, and landing.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("action_server", system, loop, logger)

        self._submit_simple_generator(self._system.action_server.arm_disarm)
        self._submit_simple_generator(self._system.action_server.flight_mode_change)
        self._submit_simple_generator(self._system.action_server.land)
        self._submit_simple_generator(self._system.action_server.reboot)
        self._submit_simple_generator(self._system.action_server.shutdown)
        self._submit_simple_generator(self._system.action_server.takeoff)
        self._submit_simple_generator(self._system.action_server.terminate)

        self._end_init()

    def arm_disarm(self) -> Optional[action_server.ArmDisarm]:
        """
        returns the current arm_disarm value
        :return: action_server.ArmDisarm ; the current arm_disarm value
        """
        return self._async_gen_data[self._system.action_server.arm_disarm]

    def flight_mode_change(self) -> Optional[action_server.FlightMode]:
        """
        returns the current flight mode
        :return: action_server.FlightMode
        """
        return self._async_gen_data[self._system.action_server.flight_mode_change]

    def land(self) -> Optional[bool]:
        """
        returns the boolean that explains if the vehicle is landing
        :return: bool ; True if the vehicle is landing, False otherwise
        """
        return self._async_gen_data[self._system.action_server.land]

    def reboot(self) -> Optional[bool]:
        """
        returns the boolean that states if the vehicle is rebooting
        :return: bool ; True if the vehicle is rebooting, False otherwise
        """
        return self._async_gen_data[self._system.action_server.reboot]

    def get_allowable_flight_modes(
        self, timeout: float = 1.0
    ) -> Optional[action_server.AllowableFlightModes]:
        """
        Returns the flight modes allowed
        :return: action_server.AllowableFlightModes ; all allowed flight modes
        """
        self._logger.info("Pulling allowable flight modes")

        flight_modes = self._submit_blocking_coroutine(
            self._system.action_server.get_allowable_flight_modes(),
            timeout=timeout,
        )

        if flight_modes is not None:
            self._logger.info("Successfully pulled allowable flight modes")
        else:
            self._logger.error(
                "Could not pull allowable flight modes! Request timed out!"
            )
        return flight_modes

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

    def shutdown(self) -> Optional[bool]:
        """
        returns if the vehicle is in the process of shutting down
        :return: bool ; True if the vehicle is shutting down, False otherwise
        """
        return self._async_gen_data[self._system.action_server.shutdown]

    def takeoff(self) -> Optional[bool]:
        """
        Returns if the vehicle is in the state of taking off
        :return: bool ; True if the vehicle is taking off, false otherwise
        """
        return self._async_gen_data[self._system.action_server.takeoff]

    def terminate(self) -> Optional[bool]:
        """
        Returns if the mission is being terminated
        :return: bool ; True if the mission is terminated, False othewise
        """
        return self._async_gen_data[self._system.action_server.terminate]
