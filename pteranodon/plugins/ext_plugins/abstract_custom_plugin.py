from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from ..abstract_plugin import AbstractPlugin


class AbstractCustomPlugin(AbstractPlugin):
    """
    Base plugin functionality, no methods required to overwrite
    """

    def __init__(
        self,
        name: str,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__(name, system, loop, logger)
        self._base_plugins = base_plugins
        self._ext_args = ext_args
