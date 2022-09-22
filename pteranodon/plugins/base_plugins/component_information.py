import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from functools import partial
from typing import List, Optional

from mavsdk import System
from mavsdk.component_information_server import FloatParam
from mavsdk.component_information_server import FloatParamUpdate

from .abstract_base_plugin import AbstractBasePlugin


class ComponentInformation(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("component_information", system, loop, logger)

        self._param_list: List[FloatParam] = []
        self._param_list_task = asyncio.ensure_future(
            self._system.component_information.access_float_params(), loop=self._loop
        )
        self._param_list_task.add_done_callback(partial(self._param_list_callback))

        self._float_param_update: FloatParamUpdate = None
        self._float_param_update_task = asyncio.ensure_future(
            self._update_float_param(), loop=self._loop
        )

    def _param_list_callback(self, task: Task) -> None:
        self._param_list = task.result()
        del self._param_list_task

    async def _update_float_param(self) -> None:
        async for curr_param_update in self._system.component_information_server.float_param():
            if curr_param_update != self._float_param_update:
                self._param_list_task = asyncio.ensure_future(
                    self._system.component_information.access_float_params(),
                    loop=self._loop,
                )
                self._param_list_task.add_done_callback(
                    partial(self._param_list_callback)
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
