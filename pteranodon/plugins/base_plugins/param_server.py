import asyncio
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Optional
from functools import partial

from mavsdk import System, param_server

from .abstract_base_plugin import AbstractBasePlugin


class ParamServer(AbstractBasePlugin):
    """
    Provide raw access to retrieve and provide server parameters.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("param_server", system, loop, logger)
        self._end_init()

    def provide_param_custom(self, name: str, value: str):
        """
        provides a custom parameter created from the name and value
        :param name: name of the custom parameter you wish to add
        :param value: String value of the parameter you wish to add
        """
        self._logger.info(
            f"Provided a custom parameter with the name {name} and a value of {value}"
        )
        self._submit_coroutine(
            self._system.param_server.provide_param_custom(name, value)
        )

    def provide_param_float(self, name: str, value: float):
        """
        provides a float parameter created from the name and value provided
        :param name: name of the float parameter you wish to add
        :param value: float value of the parameter you wish to add
        """
        self._logger.info(
            f"Provided a float parameter with the name {name} and a value of {value}"
        )
        self._submit_coroutine(
            self._system.param_server.provide_param_float(name, value)
        )

    def provide_param_int(self, name: str, value: int):
        """
        Provide an integer parameter created from the name and value provided
        :param name: name of the integer parameter you wish to add
        :param value: Integer value of the parameter you wish to add
        """
        self._logger.info(
            f"Provided an integer parameter with the name {name} and a value of {value}"
        )
        self._submit_coroutine(self._system.param_server.provide_param_int(name, value))

    def retrieve_all_params(
        self, timeout: float = 1.0
    ) -> Optional[param_server.AllParams]:
        """
        retrieves the all parameters item
        :return: param_server.AllParams or None ; If result is not none, it is the AllParams object, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_all_params")

        all_params = self._submit_blocking_coroutine(
            partial(self._system.param_server.retrieve_all_params), timeout=timeout
        )

        if all_params is not None:
            self._logger.info("Response to retrieve_all_params received")
        else:
            self._logger.error("Could not retrieve all params! Request timed out!")
        return all_params

    def retrieve_param_custom(self, name, timeout: float = 1.0) -> Optional[str]:
        """
        Retrieve the value of a custom parameter.
        :param name: Name of the custom parameter you want to retrieve.
        :return: str or None ; If the result is not None, it is the value of the requested parameter, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_param_custom")

        param_custom = self._submit_blocking_coroutine(
            partial(self._system.param_server.retrieve_param_custom, name),
            timeout=timeout,
        )

        if param_custom is not None:
            self._logger.info("Response to retrieve_param_custom received")
        else:
            self._logger.error(
                "Could not retrieve custom parameter! Request timed out!"
            )
        return param_custom

    def retrieve_param_float(self, name, timeout: float = 1.0) -> Optional[float]:
        """
        Retrieve the value of float parameter.
        :param name: Name of the float parameter you want to retrieve.
        :return: float or None ; If the result is not None, it is the value of the requested parameter, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_param_float")

        param_float = self._submit_blocking_coroutine(
            partial(self._system.param_server.retrieve_param_float, name),
            timeout=timeout,
        )

        if param_float is not None:
            self._logger.info("Response to retrieve_param_float received")
        else:
            self._logger.error("Could not retrieve float parameter! Request timed out!")
        return param_float

    def retrieve_param_int(self, name, timeout: float = 1.0) -> Optional[int]:
        """
        Retrieve the value of an integer parameter.
        :param name: Name of the integer parameter you want to retrieve.
        :return: int or None ; If the result is not None, it is the value of the requested parameter, otherwise,
         the request timed out
        """
        self._logger.info("Waiting for response to retrieve_param_int")

        param_int = self._submit_blocking_coroutine(
            partial(self._system.param_server.retrieve_param_int, name), timeout=timeout
        )

        if param_int is not None:
            self._logger.info("Response to retrieve_param_float received")
        else:
            self._logger.error("Could not retrieve float parameter! Request timed out!")
        return param_int
