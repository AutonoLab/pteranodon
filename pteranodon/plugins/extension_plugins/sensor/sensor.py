from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from ..abstract_extension_plugin import AbstractExtensionPlugin
from .abstract_sensor import AbstractSensor
from .sensor_data import SensorData


class Sensor(AbstractExtensionPlugin):
    """
    A plugin used for collecting sensors
    """

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__("sensor", system, loop, logger, base_plugins, ext_args)

        self._sensors: Dict[str, AbstractSensor] = {}

        try:
            if self._ext_args["sensors"] is not None:
                if not isinstance(self._ext_args["sensors"], list):
                    raise TypeError("Sensors must be a list of sensor objects")
                for sensor in self._ext_args["sensors"]:
                    self._sensors[sensor.name] = sensor
        except KeyError:
            pass

        self._end_init()

    @property
    def sensors(self) -> Dict[str, AbstractSensor]:
        """
        Get all of the sensors used by the drone
        :return: Dict[str, AbstractSensor] ; a dictionary of sensors using the sensor name as the key
        """
        return self._sensors

    def add_sensor(self, new_sensor: AbstractSensor) -> None:
        """
        Adds a sensor to the drone
        :param new_sensor: sensor.AbstractSensor ; An instantiated sensor
        :return: None
        """
        if new_sensor.name not in self._sensors:
            self._sensors[new_sensor.name] = new_sensor
            self._sensors[new_sensor.name].start()
        else:
            raise KeyError(f"Sensor with name: {new_sensor.name} already present")

    def remove_sensor(self, name: str) -> None:
        """
        Remove a sensor from the drone
        :param name: str ; name of the sensor to be removed
        :return: None
        """
        del self._sensors[name]

    def get_data(self, name: str) -> SensorData:
        """
        Get the data from a specific sensor
        :param name: str ; name of the sensor with desired data
        :return: SensorData ; The desired sensors data
        """
        return self._sensors[name].data

    def get_sensor(self, name: str) -> AbstractSensor:
        """
        Get a specific sensor
        :param name: str ; name of the sensor desired
        :return: AbstractSensor ; the instantiated object with the name stated
        """
        return self._sensors[name]

    def start_sensor(self, name: str) -> None:
        """
        Start a specific sensor
        :param name: str ; name of the sensor to be started
        :return: None
        """
        self._sensors[name].start()

    def stop_sensor(self, name: str) -> None:
        """
        Stop a specific sensor
        :param name: str ; name of the sensor to be stopped
        :return: None
        """
        self._sensors[name].stop()

    def start_all_sensors(self) -> None:
        """
        start all sensors
        :return: None
        """
        for _, sensor in self._sensors.items():
            sensor.start()

    def stop_all_sensors(self) -> None:
        """
        stop all sensors
        :return: None
        """
        for _, sensor in self._sensors.items():
            sensor.stop()
