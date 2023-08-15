from pteranodon import SimpleDrone
import sys
import time
import rclpy
from rclpy.node import Node
from pteranodon.plugins.meta_plugins.ros.base_plugins import (
    action_server, camera_server, 
    camera, component_information_server, core, 
    gimbal, mission_raw_server, mission_raw, mission,
    shell, telemetry,  tracking_server, transponder
)

def _main(args=None):
    #rclpy.init(args=args)
    drone = SimpleDrone("udp://:14540")

    node_telemetry = Node("telemetry")
    telemetry.register_telemetry_publishers(node_telemetry, drone.telemetry)

    # node_action = Node("action_server")
    # action_server.register_action_server_publishers(node_action, drone.action_server)
    
    # --> Returns KeyError in "python3.10/site-packages/pteranodon/plugins/plugin_manager.py", line 240
    # node_camera_server = Node("camera_server")
    # camera_server.register_camera_server_publishers(node_camera_server, drone.camera_server)

    node_camera = Node("camera")
    camera.register_camera_publishers(node_camera, drone.camera)

    # --> Returns KeyError in pteranodon plugin_manager.py
    # node_comp_info = Node("component_information_server")
    # component_information_server.register_component_info_server_publishers(node_comp_info, drone.component_information_server)

    node_core = Node("core")
    core.register_core_publishers(node_core, drone.core)

    # node_gimbal = Node("gimbal")
    # gimbal.register_gimbal_publishers(node_gimbal, drone.gimbal)

    # node_mission_raw_server = Node("mission_raw_server")
    # mission_raw_server.register_mission_raw_server_publishers(node_mission_raw_server, drone.mission_raw_server)

    # node_mission_raw = Node("mission_raw")
    # mission_raw.register_mission_raw_publishers(node_mission_raw, drone.mission_raw)

    # node_mission = Node("mission")
    # mission.register_mission_publishers(node_mission, drone.mission)

    # node_shell = Node("shell")
    # shell.register_shell_publishers(node_shell, drone.shell)

    # node_tracking_server = Node("tracking_server")
    # tracking_server.register_tracking_server_publishers(node_tracking_server, drone.tracking_server)

    # node_transponder = Node("transponder")
    # transponder.register_transponder_publishers(node_transponder, drone.transponder)

    print("registered")
    time.sleep(2)
    drone.arm()
    print("armed")
    time.sleep(2)
    drone.takeoff()
    print("takeoff")
    time.sleep(15)
    # drone.stop()
    print("loop start")
    drone.start_loop()
    print("loop done")
    time.sleep(10)
    print("teardown")
    node_telemetry.destroy_node()
    node_camera.destroy_node()
    node_core.destroy_node()
    rclpy.shutdown()
    drone.teardown()
    print("complete")


if __name__ == "__main__":
    _main()
    sys.exit()