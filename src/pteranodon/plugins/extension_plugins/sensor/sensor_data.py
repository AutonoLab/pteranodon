import datetime
from typing import Any, Optional


class SensorData:
    """
    Handles sensor updates and logging.
    """

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

    def update(self, value: Any) -> None:
        """
        Updates sensor values
        :param value: value to update sensor values to, updates value, value_type and timestamp
        :return: None
        """
        self._timestamp = datetime.datetime.now()
        self._value = value
        self._type = type(value)

    @property
    def value(self) -> Any:
        """
        Get the sensor value currently stored
        :return: Any ; the sensor value
        """
        return self._value

    @property
    def timestamp(self) -> datetime.datetime:
        """
        Get the timestamp of the last value update
        :return: datetime.datetime ; the timestamp the most recent value was updated
        """
        return self._timestamp

    @property
    def type(self) -> Any:
        """
        get the datatype of the last value update
        :return: Any ; type of the value stored
        """
        return self._type
