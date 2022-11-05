from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from ..abstract_plugin import AbstractPlugin


class AbstractCustomPlugin(AbstractPlugin):
    """
    Custom plugin functionality, no methods required to overwrite
    """

    def __init__(
        self,
        name: str,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_plugins: Dict,
        all_args: Dict,
    ) -> None:
        super().__init__(name, system, loop, logger)
        self._base_plugins = base_plugins
        self._ext_plugins = ext_plugins
        self._all_args = all_args
