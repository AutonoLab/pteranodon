from asyncio import AbstractEventLoop
from logging import Logger
from typing import Dict

from mavsdk import System

from rclpy.node import Node
import rclpy
from .base_plugins import (
    action_server,
    camera_server,
    camera,
    component_information_server,
    core,
    gimbal,
    mission_raw_server,
    mission_raw,
    mission,
    shell,
    telemetry,
    tracking_server,
    transponder,
)
from ..abstract_meta_plugin import AbstractMetaPlugin


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
        super().__init__(
            "ros", system, loop, logger, base_plugins, ext_plugins, ext_args
        )

        rclpy.init()

        # telemetry
        self._node_telemetry = Node("telemetry")
        self._telemetry = self._base_plugins["telemetry"]
        # action_server
        # self._node_action_server = Node("action_server")
        # self._action_server = self._base_plugins["action_server"]
        # camera_server
        # self.node_camera_server = Node("camera_server")
        # self._camera_server = self._base_plugins["camera_server"]
        # camera
        self._node_camera = Node("camera")
        self._camera = self._base_plugins["camera"]
        # component_info_server
        # self.node_component_info_server = Node("component_info_server")
        # self._component_info_server = self._base_plugins["component_information_server"]
        # core
        self._node_core = Node("core")
        self._core = self._base_plugins["core"]
        # gimbal
        self._node_gimbal = Node("gimbal")
        self._gimbal = self._base_plugins["gimbal"]
        # mission_raw_server
        self._node_mission_raw_server = Node("mission_raw_server")
        self._mission_raw_server = self._base_plugins["mission_raw_server"]
        # mission_raw
        self._node_mission_raw = Node("mission_raw")
        self._mission_raw = self._base_plugins["mission_raw"]
        # mission
        self._node_mission = Node("mission")
        self._mission = self._base_plugins["mission"]
        # shell
        self._node_shell = Node("shell")
        self._shell = self._base_plugins["shell"]
        # tracking_server
        self._node_tracking_server = Node("tracking_server")
        self._tracking_server = self._base_plugins["tracking_server"]
        # transponder
        self._node_transponder = Node("transponder")
        self._transponder = self._base_plugins["transponder"]

        self._plugins_list = [("telemetry", telemetry),
                           ("action_server", action_server),
                           ("camera_server", camera_server),
                           ("camera", camera),
                           ("component_info_server", component_information_server),
                           ("core", core),
                           ("gimbal", gimbal),
                           ("mission_raw_server", mission_raw_server),
                           ("mission_raw", mission_raw),
                           ("mission", mission),
                           ("shell", shell),
                           ("tracking_server", tracking_server),
                           ("transponder", transponder)]

        self._end_init()

    def start(self):
        """Starts the ROS2 node."""
        print("Starting ROS node")

        for name, lib in self._plugins_list:
            # check if plugin has register_plugin_publishers function
            if hasattr(lib, f"register_{name}_publishers"):
                register = getattr(lib, f"register_{name}_publishers")
                node = getattr(self, f"_node_{name}")
                plugin = getattr(self, f"_{name}")
                
                # plugin.register_plugin_publishers(self._node_plugin, self._plugin)
                register(node, plugin)
                print("Registered " + name)

    def stop(self):
        """Stops the ROS2 node."""
        for name, _ in self._plugins_list:
            if hasattr(self, f"_node_{name}"):
                getattr(self, f"_node_{name}").destroy_node()
        
        # rclpy.shutdown()
