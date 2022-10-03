from asyncio import AbstractEventLoop
from concurrent.futures import Future
from logging import Logger
from typing import List, Union, Any, Dict, Tuple
from functools import partial
import time

from mavsdk import System, param
from mavsdk.param import AllParams

from .abstract_base_plugin import AbstractBasePlugin


class Param(AbstractBasePlugin):
    """
    Provide raw access to get and set parameters.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("param", system, loop, logger)
        self._custom_params: Dict[str, str] = {}
        self._float_params: Dict[str, float] = {}
        self._int_params: Dict[str, int] = {}
        self._get_all_params_callback(self._loop.run_until_complete(self._system.param.get_all_params()))

    def _get_all_params_callback(self, all_params: AllParams) -> None:
        for param in all_params.custom_params:
            self._custom_params[param.name] = param.value
        for param in all_params.float_params:
            self._float_params[param.name] = param.value
        for param in all_params.int_params:
            self._int_params[param.name] = param.value

    def _set_param_callback(self, param_dict: Dict, param_name: str, param_future: Union[Future, None]) -> None:
        param_dict[param_name] = param_future.result()

    @staticmethod
    def _find_param(name: str, param_list: List[Union[param.CustomParam, param.FloatParam, param.IntParam]]) -> Any:
        for parameter in param_list:
            if name == parameter.name:
                return parameter.value
        return None

    def get_param_custom(self, name: str) -> Union[str, None]:
        """
        Get a custom parameter
        :param name: str ; name of the parameter you wish to retrieve
        :return: str ; string value of the parameter requested. None if value is not found
        """
        return self._custom_params[name]

    def get_param_float(self, name: str) -> Union[float, None]:
        """
        Get a float parameter
        :param name: str ; name of the parameter you wish to retrieve
        :return: float ; float value of the parameter requested. None if value is not found
        """
        return self._float_params[name]

    def get_param_int(self, name: str) -> Union[int, None]:
        """
        Get an integer parameter
        :param name: str ; name of the parameter you wish to retrieve
        :return: int ; integer value of the parameter requested. None if value is not found
        """
        return self._int_params[name]

    def get_all_params(self) -> Tuple[Dict[str, str], Dict[str, float], Dict[str, int]]:
        """
        Get all parameters
        :return: param.AllParams ; a collection of all parameters
        """
        return self._custom_params, self._float_params, self._int_params

    def set_param_custom(self, name: str, value: str) -> None:
        """
        Set a custom parameter
        :param name: str ; name of the parameter to be set
        :param value: str ; value of the parameter to be set
        :return: None
        """
        self._submit_coroutine(
            self._system.param.set_param_custom(name, value),
            partial(self._set_param_callback, self._custom_params, name),
        )

    def set_param_float(self, name: str, value: float) -> None:
        """
        Set a float parameter
        :param name: str ; Name of the parameter to set
        :param value: float ; Value of the parameter to be set
        :return: None
        """
        self._submit_coroutine(
            self._system.param.set_param_float(name, value),
            partial(self._set_param_callback, self._float_params, name),
        )

    def set_param_int(self, name: str, value: int) -> None:
        """
        Set an integer parameter
        :param name: str ; Name of the parameter to set
        :param value: int ; Value of the parameter to be set
        :return: None
        """
        self._submit_coroutine(
            self._system.param.set_param_int(name, value),
            partial(self._set_param_callback, self._int_params, name),
        )
