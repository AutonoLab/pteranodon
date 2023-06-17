from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from ..abstract_meta_plugin import AbstractMetaPlugin
from base_plugins import (
    action_server, camera_server, 
    camera, component_information_server, core, 
    gimbal, mission_raw_server, mission_raw, mission,
    shell, telemetry,  tracking_server, transponder
)
import rclpy
from rclpy.node import Node


class Ros(AbstractMetaPlugin):
    """Allows the user to use a ROS2 node for making calls to plugins."""

    def __init__(
        self,
        system: System,
        loop: AbstractEventLoop,
        logger: Logger,
        base_plugins: Dict,
        ext_plugins: Dict,
        ext_args: Dict,
    ) -> None:
        super().__init__("ros", system, loop, logger, base_plugins, ext_plugins, ext_args)

        self._end_init()

    def start(self):
        """Starts the ROS2 node."""
        rclpy.init()
        self.node_telemetry = Node("telemetry")
        self._telemetry = self._base_plugins["telemetry"]
        telemetry.register_telemetry_publishers(self.node, self._telemetry)

        self.node_actionserver = Node("action_server")
        self._action_server = self._base_plugins["action_server"]
        action_server.register_action_server_publishers(self.node_actionserver, self._action_server)

        # self.node_cameraserver = Node("camera_server")
        # self._camera_server = self._base_plugins["camera_server"]
        # camera_server.register_camera_server_publishers(self.node_cameraserver, self._camera_server)

        self.node_camera = Node("camera")
        self._camera = self._base_plugins["camera"]
        camera.register_camera_publishers(self.node_camera, self._camera)

        # self.node_component_info_server = Node("component_info_server")
        # self._component_info_server = self._base_plugins["component_information_server"]
        # component_information_server.register_component_info_server_publishers(self.node_component_info_server, self._component_info_server)
        
        self.node_core = Node("core")
        self._core = self._base_plugins["core"]
        core.register_core_publishers(self.node_core, self._core)

        self.node_gimbal = Node("gimbal")
        self._gimbal = self._base_plugins["gimbal"]
        gimbal.register_gimbal_publishers(self.node_gimbal, self._gimbal)

        self.node_missionrawserver = Node("mission_raw_server")
        self._mission_raw_server = self._base_plugins["mission_raw_server"]
        mission_raw_server.register_mission_raw_server_publishers(self.node_missionrawserver, self._mission_raw_server)

        self.node_missionraw = Node("mission_raw")
        self._mission_raw = self._base_plugins["mission_raw"]
        mission_raw.register_mission_raw_publishers(self.node_missionraw, self._mission_raw)

        self.node_mission = Node("mission")
        self._mission = self._base_plugins["mission"]
        mission.register_mission_publishers(self.node_mission, self._mission)

        self.node_shell = Node("shell")
        self._shell = self._base_plugins["shell"]
        shell.register_shell_publishers(self.node_shell, self._shell)

        self.node_trackingserver = Node("tracking_server")
        self._tracking_server = self._base_plugins["tracking_server"]
        tracking_server.register_tracking_server_publishers(self.node_trackingserver, self._tracking_server)
        
        self.node_transponder = Node("transponder")
        self._transponder = self._base_plugins["transponder"]
        transponder.register_transponder_publishers(self.node_transponder, self._transponder)

    def stop(self):
        """Stops the ROS2 node."""
        self.node_actionserver.destroy_node()
        # self.node_cameraserver.destroy_node()
        self.node_camera.destroy_node()
        # self.node_component_info_server.destroy_node()
        self.node_core.destroy_node()
        self.node_gimbal.destroy_node()
        self.node_mission.destroy_node()
        self.node_missionraw.destroy_node()
        self.node_missionrawserver.destroy_node()
        self.node_shell.destroy_node()
        self.node_telemetry.destroy_node()
        self.node_trackingserver.destroy_node()
        self.node_transponder.destroy_node()
        rclpy.shutdown()
