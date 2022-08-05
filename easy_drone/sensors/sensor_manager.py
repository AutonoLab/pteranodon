from typing import List

from .sensor import Sensor
from .sensor_data import SensorData


class SensorManager:
    def __init__(self, sensors: List[Sensor]) -> None:
        self._sensors = {}
        for sensor in sensors:
            self._sensors[sensor.name] = sensor

    @property
    def sensors(self) -> List[Sensor]:
        return self._sensors

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
