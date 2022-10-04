from asyncio import AbstractEventLoop
from logging import Logger
from functools import partial
from typing import List, Optional
from concurrent.futures import Future

from mavsdk import System
from mavsdk.component_information_server import FloatParamUpdate, FloatParam

from .abstract_base_plugin import AbstractBasePlugin


class ComponentInformation(AbstractBasePlugin):
    """
    Access component information such as parameters.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("component_information", system, loop, logger)

        self._param_list: List[FloatParam] = self._loop.run_until_complete(
            self._system.component_information.access_float_params()
        )

        self._float_param_update: Optional[FloatParamUpdate] = None
        self._submit_generator(self._update_float_param)

        self._end_init()

    def _param_list_callback(self, param_future: Future) -> None:
        self._param_list = param_future.result()

    async def _update_float_param(self) -> None:
        async for curr_param_update in self._system.component_information_server.float_param():
            if curr_param_update != self._float_param_update:
                self._submit_coroutine(
                    self._system.component_information.access_float_params(),
                    partial(self._param_list_callback),
                )
                self._float_param_update = curr_param_update

    def float_param(self) -> Optional[FloatParamUpdate]:
        """
        Subscribe to float param changes/updates.

        :param param: Float param definition
        :type: FloatParamUpdate
        """

        return self._float_param_update

    def access_float_params(self) -> List[FloatParam]:
        """
        List available float params.

        :return: A parameter
        :rtype: FloatParam
        """

        return self._param_list
