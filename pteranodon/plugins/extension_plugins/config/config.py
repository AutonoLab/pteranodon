from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Union, Callable, Tuple
from collections import deque
from functools import partial
import atexit
import os
import json
import configparser

from mavsdk import System

from ..abstract_extension_plugin import AbstractExtensionPlugin
from ...base_plugins.param import Param


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
        self._stack: deque[Callable] = deque()

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
        param_setter, old_param = self._get_type_call(param_name, param_value)
        self._stack.appendleft(partial(param_setter, old_param))
        param_setter(param_name, param_value)

    def _get_type_call(
        self, param_name: str, param_value: Union[float, int, str]
    ) -> Tuple[Callable, Union[float, int, str]]:
        """Returns the appropriate method call to Param to undo and the previous value of the parameter."""
        if isinstance(param_value, float):
            return self._param.set_param_float, self._param.get_param_float(param_name)
        elif isinstance(param_value, int):
            return self._param.set_param_int, self._param.get_param_int(param_name)
        elif isinstance(param_value, str):
            return self._param.set_param_custom, self._param.get_param_custom(
                param_name
            )

    def reset(self) -> None:
        """Resets the drone to its original configuration."""
        while self._stack:
            self._stack.popleft()()

    def from_file(self, file_path: str) -> None:
        """
        Sets the configuration of the drone from a file.
        Supported filetypes: json, ini, txt with param_name:param_value format"""
        # check that the file exists
        if not os.path.isfile(file_path):
            self._logger.error(f"File {file_path} not found.")
            raise FileNotFoundError(f"File {file_path} not found.")
        # check filetype
        if file_path.endswith(".json"):
            self._from_json(file_path)
        elif file_path.endswith(".ini"):
            self._from_ini(file_path)
        elif file_path.endswith(".txt"):
            self._from_txt(file_path)
        else:
            self._logger.error(f"Unsupported filetype: {file_path}")
            raise ValueError(f"Unsupported filetype: {file_path}")

    def _from_json(self, file_path: str) -> None:
        """Sets the configuration of the drone from a json file."""
        with open(file_path, "r") as f:
            config = json.load(f)
            for param_name, param_value in config.items():
                self.set_param(param_name, param_value)

    def _from_ini(self, file_path: str) -> None:
        """Sets the configuration of the drone from an ini file."""
        config = configparser.ConfigParser()
        config.read(file_path)
        for param_name, param_value in config.items():
            self.set_param(param_name, param_value)

    def _from_txt(self, file_path: str) -> None:
        """Sets the configuration of the drone from a txt file."""
        with open(file_path, "r") as f:
            for line in f:
                line = line.strip()
                if (
                    len(line) == 0
                    or line.startswith("#")
                    or line.startswith(":")
                    or line.startswith("//")
                ):
                    continue
                if ":" not in line:
                    self._logger.error(f"Invalid line: {line}")
                    raise ValueError(f"Invalid line: {line}")
                param_name, param_value = line.split(":")
                self.set_param(param_name, param_value)
