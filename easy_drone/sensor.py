from abc import ABC, abstractmethod

from .sensor_data import SensorData


class Sensor(ABC):
    def __init__(self, name):
        self._name = name
        self._sensor_data = SensorData(None)

    @property
    def name(self):
        return self._name

    def update(self, data):
        self._sensor_data.update(data)

    @abstractmethod
    def get_data(self):
        pass

    @abstractmethod
    def close(self):
        pass
