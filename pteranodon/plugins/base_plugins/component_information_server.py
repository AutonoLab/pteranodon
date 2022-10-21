from asyncio import AbstractEventLoop
from logging import Logger
from functools import partial

from mavsdk import System
from mavsdk.component_information_server import FloatParamUpdate, FloatParam

from .abstract_base_plugin import AbstractBasePlugin


class ComponentInformationServer(AbstractBasePlugin):
    """
    Provide component information such as parameters.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("component_information_server", system, loop, logger)

        self._float_param_update = None
        self._submit_simple_generator(
            self._system.component_information_server.float_param
        )
        self.register_float_param_handler = partial(
            self._register_handler,
            self._system.component_information_server.float_param,
        )

        self._end_init()

    def float_param(self) -> FloatParamUpdate:
        """
        Fetch float param updates.

        :return: A parameter update
        :rtype: FloatParamUpdate
        """

        return self._async_gen_data[
            self._system.component_information_server.float_param()
        ]

    def provide_float_param(self, param: FloatParam) -> None:
        """
        Provide a param of type float.

        :param param: Float param definition
        :type: FloatParam
        """

        self._submit_coroutine(
            self._system.component_information_server.provide_float_param(param)
        )
