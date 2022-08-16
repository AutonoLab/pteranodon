import datetime
from typing import Any, Optional, Union


class SensorData:
    def __init__(self, value: Optional[Any] = None):
        self._timestamp = datetime.datetime.now()
        self._value = value
        self._type = type(value)

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, SensorData):
            return self.value == other.value and self.timestamp == other.timestamp
        return False

    def __repr__(self) -> str:
        return f"SensorData({self._timestamp},{self._type},{self._value})"

    def __str__(self) -> str:
        return f"({self._timestamp},{self._type},{self._value})"

    def update(self, value: Any):
        self._timestamp = datetime.datetime.now()
        self._value = value
        self._type = type(value)

    @property
    def value(self) -> Any:
        return self._value

    @property
    def timestamp(self) -> datetime.datetime:
        return self._timestamp

    @property
    def type(self) -> Any:
        return self._type
