from abc import ABC, abstractmethod
from typing import Any, Union, Optional
from threading import Thread
import time

from .sensor_data import SensorData


class Sensor(ABC):
    def __init__(self, sensor_name: str, poll_rate: Optional[float] = None):
        """
        :param name: A string to represent the sensor
        :param poll_rate: The rate to poll the sensor at (in Hz)
        """
        self._name = sensor_name
        # convert to ms for a direct sleep call
        self._poll_rate = None if poll_rate is None else 1.0 / poll_rate
        self._sensor_data = SensorData()
        self._stopped = False
        self._update_thread = Thread(name=f"{self._name}_update_thread", target=self._run, args=())

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, Sensor):
            return self.data == other.data and self.name == other.name and self._poll_rate == other._poll_rate and \
                   self._update_thread == other._update_thread
        return False

    def _run_timed(self) -> None:
        while not self._stopped:
            start_time = time.perf_counter()
            self.update_data()
            elapsed_time = time.perf_counter() - start_time

            if elapsed_time <= self._poll_rate:
                time.sleep(self._poll_rate - elapsed_time)

    def _run_untimed(self) -> None:
        while not self._stopped:
            self.update_data()

    def _run(self) -> None:
        try:
            if self._poll_rate is None:
                self._run_untimed()
            else:
                self._run_timed()
        finally:
            self.teardown()

    def start(self) -> None:
        self._update_thread.start()

    def stop(self) -> None:
        self._stopped = True
        self._update_thread.join()

    @property
    def name(self) -> str:
        return self._name

    @property
    def poll_rate(self) -> Union[float, None]:
        return self._poll_rate

    @poll_rate.setter
    def poll_rate(self, rate: float) -> None:
        self._poll_rate = 1.0 / rate

    @property
    def data(self) -> SensorData:
        return self._sensor_data

    @data.setter
    def data(self, new_data: Any) -> None:
        self._sensor_data.update(new_data)

    @abstractmethod
    def update_data(self):
        """
        A method which takes no parameters and has no return value. This handles updating the sensor data inside of the
        data field. This method gets called repeatedly inside of the sensor update loop at the rate defined by
        poll_rate.
        """
        pass

    @abstractmethod
    def teardown(self):
        """
        This method should define any final behavior of the sensor. This will mean closing sensor pipeline, file, etc..
        """
        pass
