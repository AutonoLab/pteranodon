import asyncio
from asyncio import AbstractEventLoop, Task
from logging import Logger
from typing import List, Union, Any
from functools import partial

from mavsdk import System, param

from .abstract_base_plugin import AbstractBasePlugin


class Param(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("param", system, loop, logger)
        self._all_params = None
        self._custom_params = None
        self._float_params = None
        self._int_params = None
        self._param_task = None
        self.refresh()

    def _update_params_callback(self, task: Task) -> None:
        all_params = task.result()
        self._all_params = all_params
        try:
            self._custom_params = self._all_params.custom_params
        except AttributeError:
            pass
        self._float_params = self._all_params.float_params
        self._int_params = self._all_params.int_params

    def _set_param_callback(self, _: Union[Task, None]) -> None:
        # can use a Union parameter for the callback since the task itself is not edited
        self._param_task = asyncio.ensure_future(self._system.param.get_all_params(), loop=self._loop)
        self._param_task.add_done_callback(partial(self._update_params_callback))

    @staticmethod
    def _find_param(name: str, param_list: List) -> Any:
        for parameter in param_list:
            if name == parameter.name:
                return parameter.value
        return None

    def get_param(self, name: str, search_custom=False, search_float=False, search_int=False) -> Any:
        """
        Method which gets the value of a parameter found in any of the parameter lists. The order in which the parameter
        value is returned is -> custom, float, int. Thus, if a parameter is found in custom that value will be returned
        before the float or int parameters are searched
        :param name: The name of the parameters in string form
        :param search_custom: Boolean, true if the custom parameters should be searched
        :param search_float: Boolean, true if the float parameters should be searched
        :param search_int: Boolean, true if the int parameters should be searched
        :returns: The value of the parameter if found, otherwise None
        """
        param_val = None
        if search_custom:
            param_val = Param._find_param(name, self._custom_params)
        elif param_val is None and search_float:
            param_val = Param._find_param(name, self._float_params)
        elif param_val is None and search_int:
            param_val = Param._find_param(name, self._int_params)
        return param_val

    def get_param_custom(self, name: str) -> Union[str, None]:
        return self.get_param(name, True, False, False)

    def get_param_float(self, name: str) -> Union[float, None]:
        return self.get_param(name, False, True, False)

    def get_param_int(self, name: str) -> Union[int, None]:
        return self.get_param(name, False, False, True)

    def get_all_params(self) -> param.AllParams:
        return self._all_params

    def set_param_custom(self, name: str, value: str) -> None:
        try:
            param_task = super().submit_task(
                asyncio.ensure_future(self._system.param.set_param_custom(name, value), loop=self._loop)
            )
            param_task.add_done_callback(partial(self._set_param_callback))
        except AttributeError:
            self._logger.error(f"Unable to set param: {name} to {value}. No attribute set_param_custom")

    def set_param_float(self, name: str, value: float) -> None:
        param_task = super().submit_task(
            asyncio.ensure_future(self._system.param.set_param_float(name, value), loop=self._loop)
        )
        param_task.add_done_callback(partial(self._set_param_callback))

    def set_param_int(self, name: str, value: int) -> None:
        param_task = super().submit_task(
            asyncio.ensure_future(self._system.param.set_param_int(name, value), loop=self._loop)
        )
        param_task.add_done_callback(partial(self._set_param_callback))

    def refresh(self) -> None:
        self._set_param_callback(None)
