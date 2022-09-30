import asyncio
from asyncio import AbstractEventLoop
from logging import Logger

from mavsdk import System
from mavsdk.component_information_server import FloatParamUpdate
from mavsdk.component_information_server import FloatParam

from .abstract_base_plugin import AbstractBasePlugin


class ComponentInformationServer(AbstractBasePlugin):
    """
    Provide component information such as parameters.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("component_information_server", system, loop, logger)

        self._float_param_update = None
        self._float_param_update_task = asyncio.ensure_future(
            self._update_float_param(), loop=self._loop
        )

    async def _update_float_param(self) -> None:
        async for curr_param_update in self._system.component_information_server.float_param():
            if curr_param_update != self._float_param_update:
                self._float_param_update = curr_param_update

    def float_param(self) -> FloatParamUpdate:
        """
        Subscribe to float param updates.

        :return: A parameter update
        :rtype: FloatParamUpdate
        """

        return self._float_param_update

    def provide_float_param(self, param: FloatParam) -> None:
        """
        Provide a param of type float.

        :param param: Float param definition
        :type: FloatParam
        """

        super().submit_coroutine(
            self._system.component_information_server.provide_float_param(param)
        )
