import datetime
from typing import Any, Optional


class SensorData:
    def __init__(self, data: Optional[Any] = None):
        self._timestamp = datetime.datetime.now()
        self._data = data
        self._type = type(data)

    def __eq__(self, other: "SensorData") -> bool:
        if isinstance(other, SensorData):
            return self.data == other.data and self.timestamp == other.timestamp
        return False

    def __repr__(self) -> str:
        return f"SensorData({self._timestamp},{self._type},{self._data})"

    def __str__(self) -> str:
        return f"({self._timestamp},{self._type},{self._data})"

    def update(self, data: Any):
        self._timestamp = datetime.datetime.now()
        self._data = data
        self._type = type(data)

    @property
    def data(self) -> Any:
        return self._data

    @property
    def timestamp(self) -> datetime.datetime:
        return self._timestamp

    @property
    def type(self) -> Any:
        return self._type
