from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional

from mavsdk import System, follow_me

from .abstract_base_plugin import AbstractBasePlugin


class FollowMe(AbstractBasePlugin):
    """
    Allow users to command the vehicle to follow a specific target. The target is provided as a GPS coordinate and altitude.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("follow_me", system, loop, logger)
        self._is_active: Optional[bool] = None
        self._last_location: Optional[follow_me.TargetLocation] = None
        self._config: Optional[follow_me.Config] = None

        # only gotta wait on async tasks to get the data non-async since we store the parameters in method calls
        self._is_active = self._loop.run_until_complete(
            self._system.follow_me.is_active()
        )
        self._last_location = self._loop.run_until_complete(
            self._system.follow_me.get_last_location()
        )
        self._config_task = self._loop.run_until_complete(
            self._system.follow_me.get_config()
        )

        self._end_init()

    def get_config(self) -> Optional[follow_me.Config]:
        """
        Get current configuration
        :return: follow_me.Config ; The current configuration
        """
        return self._config

    def get_last_location(self) -> Optional[follow_me.TargetLocation]:
        """
        Get the last location of the target
        :return: follow_me.TargetLocation ; The last location of the target
        """
        return self._last_location

    def is_active(self) -> Optional[bool]:
        """
        Check if FollowMe is active
        :return: boolean ; True if FollowMe is active, False otherwise
        """
        return self._is_active

    def set_config(self, config: follow_me.Config) -> None:
        """
        Apply configuration by sending it to the system
        :param config: follow_me.Config ; The configuration to be set
        :return: None
        """
        self._submit_coroutine(self._system.follow_me.set_config(config))
        self._config = config

    def set_target_location(self, location: follow_me.TargetLocation) -> None:
        """
        Set the location of the moving target
        :param location: follow_me.TargetLocation ; The target location to be set
        :return: None
        """
        self._submit_coroutine(self._system.follow_me.set_target_location(location))
        self._last_location = location

    def start(self) -> None:
        """
        Start FollowMe mode
        :return: None
        """
        self._submit_coroutine(self._system.follow_me.start())
        self._is_active = True

    def stop(self) -> None:
        """
        Stop FollowMe mode
        :return: None
        """
        self._submit_coroutine(self._system.follow_me.start())
        self._is_active = False

    '''def fly_drone(self):
        self._system.connect("udp://:14540")
        self._system.action.arm()
        self._system.action.'''
