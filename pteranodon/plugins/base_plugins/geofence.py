from asyncio import AbstractEventLoop
from logging import Logger
from typing import List

from mavsdk import System
from mavsdk.geofence import Polygon

from .abstract_base_plugin import AbstractBasePlugin


class Geofence(AbstractBasePlugin):
    """
    Enable setting a geofence.
    """

    def __init__(self, system: System, loop: AbstractEventLoop, logger: Logger) -> None:
        super().__init__("geofence", system, loop, logger)

    def clear_geofence(self) -> None:
        """
        Clears the current geofences present on the drone.
        """
        self._logger.info("Cleared all geofences onboard the system")
        super().submit_coroutine(self._system.geofence.clear_geofence())

    def upload_geofence(self, polygons: List[Polygon]) -> None:
        """
        Uploads the geofence to become active on the drone
        :param polygons: A list of mavsdk.geofence.Polygon objects which form the geofence
        """
        self._logger.info(f"Uploading {len(polygons)} geofences to the drone")

        super().submit_coroutine(self._system.geofence.upload_geofence(polygons))
