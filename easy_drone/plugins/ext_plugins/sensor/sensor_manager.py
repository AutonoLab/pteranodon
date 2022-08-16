from asyncio import AbstractEventLoop
from logging import Logger
from typing import List

from mavsdk import System

from ..abstract_custom_plugin import AbstractCustomPlugin, PluginManager
from .sensor import Sensor
from .sensor_data import SensorData


class SensorManager(AbstractCustomPlugin):
    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger, base_plugins: Dict, ext_args: Dict)\
            -> None:
        super().__init__("sensor", system, loop, logger, base_plugins, ext_args)

        self._sensors = {}
        
        if self._ext_args["sensors"] is not None:
            for sensor in self._ext_args["sensors"]:
                self._sensors[sensor.name] = sensor

    @property
    def sensors(self) -> List[Sensor]:
        return self._sensors

    def add_sensor(self, new_sensor: Sensor) -> None:
        if new_sensor.name not in self._sensors:
            self._sensors[new_sensor.name] = new_sensor
            self._sensors[new_sensor.name].start()
        else:
            raise KeyError(f"Sensor with name: {new_sensor.name} already present")

    def remove_sensor(self, name: str) -> None:
        del self._sensors[name]

    def get_data(self, name: str) -> SensorData:
        return self._sensors[name].data

    def get_sensor(self, name: str) -> Sensor:
        return self._sensors[name]

    def start_sensor(self, name: str) -> None:
        self._sensors[name].start()

    def stop_sensor(self, name: str) -> None:
        self._sensors[name].stop()

    def start_all_sensors(self) -> None:
        for sensor in self._sensors:
            sensor.start()

    def stop_all_sensors(self) -> None:
        for sensor in self._sensors:
            sensor.stop()
