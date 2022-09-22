import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional
from threading import Condition

from mavsdk import System, param_server

from .abstract_base_plugin import AbstractBasePlugin


class ParamServer(AbstractBasePlugin):
    """
    Provide raw access to retrieve and provide server parameters.
    """
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("param_server", system, loop, logger)

    def provide_param_custom(self, name: str, value: str):
        """
        provides a custom parameter created from the name and value
        :param name: name of the custom parameter you wish to add
        :param value: String value of the parameter you wish to add
        """
        self._logger.info(f"Provided a custom parameter with the name {name} and a value of {value}")
        super().submit_task(
            asyncio.ensure_future(self._system.param_server.provide_param_custom(name, value), loop=self._loop)
        )

    def provide_param_float(self, name: str, value: float):
        """
        provides a float parameter created from the name and value provided
        :param name: name of the float parameter you wish to add
        :param value: float value of the parameter you wish to add
        """
        self._logger.info(f"Provided a float parameter with the name {name} and a value of {value}")
        super().submit_task(
            asyncio.ensure_future(self._system.param_server.provide_param_float(name, value), loop=self._loop)
        )

    def provide_param_int(self, name: str, value: int):
        """
        Provide an integer parameter created from the name and value provided
        :param name: name of the integer parameter you wish to add
        :param value: Integer value of the parameter you wish to add
        """
        self._logger.info(f"Provided an integer parameter with the name {name} and a value of {value}")
        super().submit_task(
            asyncio.ensure_future(self._system.param_server.provide_param_int(name, value), loop=self._loop)
        )

    def retrieve_all_params(self) -> Optional[param_server.AllParams]:
        """
        retrieves the all parameters item
        :return: param_server.AllParams or None ; If result is not none, it is the AllParams object, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_all_params")

        retrieve_all_params_task = asyncio.ensure_future(
            self._system.param_server.retrieve_all_params(), loop=self._loop
        )
        done_condition = Condition()
        retrieve_all_params_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        try:
            x = retrieve_all_params_task.result()
            self._logger.info("Response to retrieve_all_params received")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Could not retrieve all params! Request timed out!")
            return None

    def retrieve_param_custom(self, name) -> Optional[str]:
        """
        Retrieve the value of a custom parameter.
        :param name: Name of the custom parameter you want to retrieve.
        :return: str or None ; If the result is not None, it is the value of the requested parameter, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_param_custom")

        retrieve_param_custom_task = asyncio.ensure_future(
            self._system.param_server.retrieve_param_custom(name), loop=self._loop
        )
        done_condition = Condition()
        retrieve_param_custom_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        try:
            x = retrieve_param_custom_task.result()
            self._logger.info("Response to retrieve_param_custom received")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Could not retrieve custom parameter! Request timed out!")
            return None

    def retrieve_param_float(self, name) -> Optional[float]:
        """
        Retrieve the value of float parameter.
        :param name: Name of the float parameter you want to retrieve.
        :return: float or None ; If the result is not None, it is the value of the requested parameter, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_param_float")

        retrieve_param_float_task = asyncio.ensure_future(
            self._system.param_server.retrieve_param_float(name), loop=self._loop
        )
        done_condition = Condition()
        retrieve_param_float_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        try:
            x = retrieve_param_float_task.result()
            self._logger.info("Response to retrieve_param_float received")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Could not retrieve float parameter! Request timed out!")
            return None

    def retrieve_param_int(self, name) -> Optional[int]:
        """
        Retrieve the value of an integer parameter.
        :param name: Name of the integer parameter you want to retrieve.
        :return: int or None ; If the result is not None, it is the value of the requested parameter, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_param_int")

        retrieve_param_int_task = asyncio.ensure_future(
            self._system.param_server.retrieve_param_int(name), loop=self._loop
        )
        done_condition = Condition()
        retrieve_param_int_task.add_done_callback(lambda _: done_condition.notify())
        done_condition.wait(1.0)

        try:
            x = retrieve_param_int_task.result()
            self._logger.info("Response to retrieve_param_float received")
            return x
        except asyncio.InvalidStateError:
            self._logger.error("Could not retrieve float parameter! Request timed out!")
            return None
