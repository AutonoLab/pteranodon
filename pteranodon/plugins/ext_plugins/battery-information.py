import time
from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict, Optional
from collections import deque

import numpy as np
from mavsdk import System

from .abstract_custom_plugin import AbstractCustomPlugin


class BatteryInfo(AbstractCustomPlugin):
    """
    Generates battery usage information
    """

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__(
            "battery_information", system, loop, logger, base_plugins, ext_args
        )
        # pulling plugins
        self._telemetry = self._base_plugins["telemetry"]
        self._param = self._base_plugins["param"]

        # pulling args
        if self._ext_args["battery_information"] is not None:
            self._window_size = self._ext_args["battery_information"]

        # Submitting Generators
        self._submit_generator(self._battery_updates)

        # Private vars for calculation
        self._capacity = self._param.get_param_float("BAT1_CAPACITY")
        self._num_cells = self._param.get_param_int("BAT1_N_CELLS")

        # Private Datastructures
        self._window: deque = deque(maxlen=self._window_size)

    async def _battery_updates(self) -> None:
        async for battery in self._system.telemetry.battery():
            self._window.append((battery, time.time()))

    def get_instantaneous_wattage(self) -> Optional[float, None]:
        """
        Get the Current*Voltage value, or instantaneous wattage
        :return: float : Instantaneous battery usage, Current*Voltage, None if battery has not been polled yet
        """
        if len(self._window) > 0:
            batt_info = self._window[-1]
            return self._param.get_param_float("BAT1_A_PER_V") * (
                batt_info.voltage_v**2
            )
        else:
            return None

    def _average_voltage(self, window) -> float:
        # Probably could be done with numpy mean with setting axes. Testing would need to encompass this file to tell
        sum_voltage = 0
        for batt_and_time in window:
            sum_voltage += batt_and_time[0].voltage_v
        return sum_voltage / self._window_size

    def battery_percent_usage_over_time(self) -> float:
        """
        Get the battery percentage usage over a timeframe specified by battery poll rate and window size
        :return: float ; the battery percentage over time
        """
        delta_percentage = (
            self._window[-1][0].remaining_percent - self._window[0][0].remaining_percent
        )
        delta_time = self._window[-1][1] - self._window[0][1]
        return delta_percentage / delta_time

    def batter_usage_over_time(self) -> float:
        """
        The "amount" of mili-amp hours consumed during a time frame specified by battery poll rate and window size
        :return: float ; battery usage over time in mili-Amp hours
        """
        return self.battery_percent_usage_over_time() * self._capacity

    def estimated_time_remaining_linear(self) -> float:
        """
        Returns the time remaining using a linear regression, may be inaccurate based on battery type discharge rates
        and limits on window size
        :return: float : the time remaining in seconds
        """
        percentage_array = np.empty(0)
        time_array = np.empty(0)
        for x in self._window:
            percentage_array = np.append(percentage_array, x[0].remaining_percent)
            time_array = np.append(time_array, x[1])
        time_array = time_array - time_array[0]
        time_array_fixed = np.vstack([time_array, np.ones(len(time_array))]).T
        slope, power_intercept = np.linalg.lstsq(time_array_fixed, percentage_array, rcond=None)[0]
        time_intercept = -power_intercept / slope
        return time_intercept

    def total_power_consumed_to_time(self) -> float:
        """
        The mili-amp hours "consumed" as recent as the latest poll from the battery.
        :return: float ; the mili-amp hours "consumed"
        """
        return self._capacity - (self._window[-1][0].remaining_percent * self._capacity)
