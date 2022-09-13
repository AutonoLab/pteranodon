import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import List, Any
from mavsdk import System, param_server
from .abstract_base_plugin import AbstractBasePlugin


class ParamServer(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("param_server", system, loop, logger)

        self._all_params: param_server.AllParams = None
        self._custom_params: List[param_server.CustomParam] = []
        self._float_params: List[param_server.FloatParam] = []
        self._int_params: List[param_server.IntParam] = []

    def _update_all_params(self):
        """
        Updates the _all_params object
        """
        self._all_params = param_server.AllParams(self._custom_params, self._float_params, self._int_params)

    def provide_param_custom(self, name, value):
        """
        Creates a custom parameter and appends it to the _custom_params list, updates the existing AllParams item
        :param name: name of the custom parameter you wish to add
        :param value: String value of the parameter you wish to add
        """
        self._logger.info(f"Provided a custom parameter with the name {name} and a value of {value}")
        self._custom_params.append(param_server.CustomParam(name, value))
        self._update_all_params()
        super().submit_task(
            asyncio.ensure_future(self._system.param_server.provide_param_custom(name, value), loop=self._loop)
        )

    def provide_param_float(self, name, value):
        """
        Creates a float parameter and appends it to the _float_params list, updates the existing AllParams item
        :param name: name of the float parameter you wish to add
        :param value: float value of the parameter you wish to add
        """
        self._logger.info(f"Provided a float parameter with the name {name} and a value of {value}")
        self._float_params.append(param_server.FloatParam(name, value))
        self._update_all_params()
        super().submit_task(
            asyncio.ensure_future(self._system.param_server.provide_param_float(name, value), loop=self._loop)
        )

    def provide_param_int(self, name, value):
        """
        Creates an integer parameter and appends it to the _int_params list, updates the existing AllParams item
        :param name: name of the integer parameter you wish to add
        :param value: Integer value of the parameter you wish to add
        """
        self._logger.info(f"Provided an integer parameter with the name {name} and a value of {value}")
        self._int_params.append(param_server.IntParam(name, value))
        self._update_all_params()
        super().submit_task(
            asyncio.ensure_future(self._system.param_server.provide_param_int(name, value), loop=self._loop)
        )

    @staticmethod
    def _find_param(name: str, param_list: List) -> Any:
        """
        Finds a parameter based on the name of the desired parameter.
        :param name: name of the desired parameter.
        :param param_list: The list of all parameters of the desired type.
        :return: returns the value of the desired parameter.
        """
        for param in param_list:
            if name == param.name:
                return param.value
        return None

    def retrieve_all_params(self) -> param_server.AllParams:
        """
        Retrieve the most up to date collection of all parameters.
        :return: returns the param_server.AllParams, a collection of all parameters.
        """
        self._all_params = param_server.AllParams(self._custom_params, self._float_params, self._int_params)
        return self._all_params

    def retrieve_param_custom(self, name) -> str:
        """
        Retrieve the value of a custom parameter.
        :param name: Name of the custom parameter you want to retrieve.
        :return: returns the string value of the parameter.
        """
        param_val = ParamServer._find_param(name, self._custom_params)
        return param_val

    def retrieve_param_float(self, name) -> float:
        """
        Retrieve the value of float parameter.
        :param name: Name of the float parameter you want to retrieve.
        :return: returns the float value of the parameter.
        """
        param_val = ParamServer._find_param(name, self._float_params)
        return param_val

    def retrieve_param_int(self, name) -> int:
        """
        Retrieve the value of an integer parameter.
        :param name: Name of the integer parameter you want to retrieve.
        :return: returns the integer value of the parameter.
        """
        param_val = ParamServer._find_param(name, self._int_params)
        return param_val
