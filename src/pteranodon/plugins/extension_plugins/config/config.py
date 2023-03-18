from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Union, Callable, Tuple, Deque
from collections import deque
from functools import partial
import atexit
import os
import configparser

from mavsdk import System

from ..abstract_extension_plugin import AbstractExtensionPlugin
from ...base_plugins.param import Param
from ...base_plugins.telemetry import Telemetry


class Config(AbstractExtensionPlugin):
    """Allows the user to change the configuration of the drone, maintains a stack of changes to be undone on exit."""

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__("config", system, loop, logger, base_plugins, ext_args)

        self._param: Param = self._base_plugins["param"]
        self._telemetry: Telemetry = self._base_plugins["telemetry"]
        self._stack: Deque[Callable] = deque()

        atexit.register(self.reset)

        try:
            if self._ext_args["config_file"] is not None:
                self._config_file = self._ext_args["config_file"]
                self.from_file(self._config_file)
        except KeyError:
            pass

        self._end_init()

    def set_param(self, param_name: str, param_value: Union[float, int, str]) -> None:
        """Sets the parameter of the drone and adds it to the stack of changes to be undone on exit."""
        try:
            param_setter, old_param = self._get_type_call(
                param_name.lower(), param_value
            )
        except (
            KeyError
        ):  # checks if the parameter is all caps vs. all lower case, no mixed in PX4
            param_setter, old_param = self._get_type_call(
                param_name.upper(), param_value
            )
        if param_setter is None:
            self._logger.error(
                f"Unsupported type: {type(param_value)}. Supported types: float, int, str"
            )
            raise TypeError(
                f"Unsupported type: {type(param_value)}. Supported types: float, int, str"
            )
        self._stack.appendleft(partial(param_setter, old_param))
        param_setter(param_name, param_value)

    def _get_type_call(
        self, param_name: str, param_value: Union[float, int, str]
    ) -> Tuple[Union[Callable, None], Union[float, int, str, None]]:
        """Returns the appropriate method call to Param to undo and the previous value of the parameter."""
        if isinstance(param_value, float):
            return self._param.set_param_float, self._param.get_param_float(param_name)
        if isinstance(param_value, int):
            return self._param.set_param_int, self._param.get_param_int(param_name)
        if isinstance(param_value, str):
            return self._param.set_param_custom, self._param.get_param_custom(
                param_name
            )
        return None, None

    def reset(self) -> None:
        """Resets the drone to its original configuration."""
        while self._stack:
            self._stack.popleft()()

    def from_file(self, file_path: str) -> None:
        """
        Sets the configuration of the drone from a file.
        Supported filetypes: cfg
        """
        # check that the file exists
        if not os.path.isfile(file_path):
            self._logger.error(f"File {file_path} not found.")
            raise FileNotFoundError(f"File {file_path} not found.")
        # check filetype
        if file_path.endswith(".cfg"):
            self._from_cfg(file_path)
        else:
            self._logger.error(f"Unsupported filetype: {file_path}")
            raise ValueError(f"Unsupported filetype: {file_path}")

    @staticmethod
    def _convert_str(param_value: str, section: str) -> Union[float, int, str]:
        """Converts a string to a float, int, or str."""
        if section == "int_params":
            return int(param_value)
        if section == "float_params":
            return float(param_value)
        return param_value

    def _from_cfg(self, file_path: str) -> None:
        """Sets the configuration of the drone from a cfg file."""
        config = configparser.ConfigParser()
        config.read(file_path)
        for section in config.sections():
            if section not in [
                "int_params",
                "float_params",
                "custom_params",
                "telemetry",
            ]:
                raise ValueError(f"Invalid section: {section}")

            if section != "telemetry":
                for key, value in config.items(section):
                    try:
                        param_val = Config._convert_str(value, section)
                        self.set_param(key, param_val)
                        self._logger.info(f"Set {key} to {param_val}")
                    except KeyError:
                        self._logger.warning(f"Invalid parameter name: {key}")
            else:
                for key, value in config.items(section):
                    try:
                        attr = getattr(self._telemetry, key)
                        attr(value)
                    except AttributeError as exc:
                        self._logger.error(f"Invalid telemetry attribute: {key}")
                        self._logger.error(exc)
