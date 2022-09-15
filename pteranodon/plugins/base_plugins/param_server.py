import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional
from threading import Condition

from mavsdk import System, param_server

from .abstract_base_plugin import AbstractBasePlugin


class ParamServer(AbstractBasePlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("param_server", system, loop, logger)

    def provide_param_custom(self, name, value):
        """
        Creates a custom parameter and appends it to the _custom_params list, updates the existing AllParams item
        :param name: name of the custom parameter you wish to add
        :param value: String value of the parameter you wish to add
        """
        self._logger.info(f"Provided a custom parameter with the name {name} and a value of {value}")
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
        super().submit_task(
            asyncio.ensure_future(self._system.param_server.provide_param_int(name, value), loop=self._loop)
        )

    def retrieve_all_params(self) -> Optional[param_server.AllParams]:
        """
        retrieves the all parameters item
        :return: param_server.AllParams ; details on all parameters
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
        :return: returns the string value of the parameter.
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
        :return: returns the float value of the parameter.
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
        :return: returns the integer value of the parameter.
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
