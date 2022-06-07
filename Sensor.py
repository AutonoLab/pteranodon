from abc import abstractmethod
from pkgutil import get_data
from typing_extensions import Self

from click import pass_context
from SensorData import SensorData
import datetime


class Sensor:
    def __init__(self, id):
        self.id = id
        self.sensor_data = SensorData(None)

    def update(self, data):
        self.sensor_data.update(data)

    @abstractmethod
    def get_data():
        pass

    @abstractmethod
    def close():
        pass
