from asyncio import AbstractEventLoop
from logging import Logger
from typing import Tuple, Dict

from mavsdk import System
from mavsdk.telemetry import PositionVelocityNed, EulerAngle, Position
from mavsdk.geofence import Point, Polygon
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
from numpy import arctan2, degrees, sqrt, cos, sin, radians

from ..abstract_extension_plugin import AbstractExtensionPlugin
from ...base_plugins.telemetry import Telemetry


class Relative(AbstractExtensionPlugin):
    """
    Enables movement relative to the drone compared to absolute movement.
    """

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__("relative", system, loop, logger, base_plugins, ext_args)
        self._min_follow_distance = 10.0

        try:
            if self._ext_args["min_follow_distance"] is not None:
                self._min_follow_distance = self._ext_args["min_follow_distance"]
        except KeyError:
            pass

        self._telemetry: Telemetry = self._base_plugins["telemetry"]

        self._end_init()

    @property
    def min_follow_distance(self) -> float:
        """
        :return: The minimum following distance that is currently being used
        """
        return self._min_follow_distance

    @min_follow_distance.setter
    def min_follow_distance(self, dist: float):
        """
        :param dist: The new minimum distance that the drone should use in the maneuver_to method
        """
        self._min_follow_distance = dist

    def maneuver_to(
        self,
        front: float,
        right: float,
        down: float,
        on_dimensions: Tuple[bool, bool, bool] = (True, True, True),
        test_min: bool = False,
    ):
        """
        A movement command for moving relative to the drones current position. The front direction is aligned directly with
        the drones front as defined in the configuration.

        :param front: Relative distance in front of drone
        :type front: float
        :param right: Relative distance to the right of drone
        :type right: float
        :param down: Relative distance below the drone
        :type down: float
        :param on_dimensions: A tuple of 3 boolean values. In order, they represent if the drone will move
        (front, right, down). If set to False the drone will not move in that direction
        :type on_dimensions: Tuple[bool, bool, bool]
        """
        self._submit_coroutine(
            self._maneuver_to(front, right, down, on_dimensions, test_min)
        )

    async def _maneuver_to(
        self,
        front: float,
        right: float,
        down: float,
        on_dimensions=(True, True, True),
        test_min=False,
    ) -> None:
        if test_min:
            total_distance = sqrt(pow(front, 2) + pow(right, 2) + pow(down, 2))
            if total_distance < self._min_follow_distance:
                await self._system.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0, 0, 0, 0)
                )
        await self._maneuver_with_ned(front, right, down, on_dimensions)

    async def _maneuver_with_ned(
        self,
        front: float,
        right: float,
        down: float,
        on_dimensions: Tuple = (True, True, True),
    ) -> None:
        # zero out dimensions that will not be moved
        front = 0.0 if not on_dimensions[0] else front
        right = 0.0 if not on_dimensions[1] else right
        down = 0.0 if not on_dimensions[2] else down

        # get current position
        task_opt = self._telemetry.position_velocity_ned
        task2_opt = self._telemetry.attitude_euler
        if task_opt is None:
            return None

        task: PositionVelocityNed = task_opt
        task2: EulerAngle = task2_opt

        current_pos = task.position

        # get angle of rotation
        angle = task2.yaw_deg
        angle_of_rotation = radians(angle)

        # convert FRD to NED
        relative_north = right * sin(angle_of_rotation) + front * cos(angle_of_rotation)
        relative_east = right * cos(angle_of_rotation) - front * sin(angle_of_rotation)

        # get angle of yaw
        yaw = degrees(arctan2(relative_east, relative_north))

        # add offset to current position
        north = relative_north + current_pos.north_m
        east = relative_east + current_pos.east_m
        down = down + current_pos.down_m

        new_pos = PositionNedYaw(north, east, down, yaw)
        await self._system.offboard.set_position_ned(new_pos)

    def create_geofence(self, distance: float) -> None:
        """
        Creates a relative inclusive geofence around the drones home coordinates. The geofence is defined as a square
        where the distance parameter is equal to half the side length.
        :param distance: The meters from home the drone can maneuver
        :return: None
        """
        self._submit_coroutine(self._create_geofence(distance))

    async def _create_geofence(self, distance: float) -> None:

        home_opt = self._telemetry.home
        if home_opt is None:
            return None

        home: Position = home_opt

        latitude = home.latitude_deg
        longitude = home.longitude_deg

        # source for magic number
        # https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of
        # -meters
        offset = distance * (1 / 111111)

        # Define your geofence boundary
        p1 = Point(latitude - offset, longitude - offset)
        p2 = Point(latitude + offset, longitude - offset)
        p3 = Point(latitude + offset, longitude + offset)
        p4 = Point(latitude - offset, longitude + offset)

        # Create a polygon object using your points
        polygon = Polygon([p1, p2, p3, p4], Polygon.FenceType.INCLUSION)

        await self._system.geofence.upload_geofence([polygon])
