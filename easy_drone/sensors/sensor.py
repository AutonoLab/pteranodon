from abc import ABC, abstractmethod
from typing import Any

from .sensor_data import SensorData


class Sensor(ABC):
    def __init__(self, name):
        self._name = name
        self._sensor_data = SensorData()

    @property
    def name(self) -> str:
        return self._name

    @property
    def data(self) -> SensorData:
        return self._sensor_data

    @abstractmethod
    def update_data(self):
        pass

    @abstractmethod
    def get_data(self):
        pass

    @abstractmethod
    def close(self):
        pass
