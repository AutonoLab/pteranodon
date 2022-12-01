from pteranodon import SimpleDrone
import time
from mavsdk.gimbal import ControlMode, GimbalMode

input("\ndrone = SimpleDrone('udp://:14540')\n")
drone = SimpleDrone("udp://:14540")

time.sleep(2)
input("\ndrone.arm()\n")
drone.arm()

time.sleep(2)
input("\ndrone.takeoff()\n")
drone.takeoff()


time.sleep(2)
input("\ndrone.gimbal.take_control(ControlMode.PRIMARY)\n")
drone.gimbal.take_control(ControlMode.PRIMARY)

time.sleep(2)
input("\ndrone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)\n")
drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)

time.sleep(2)
input("\ndrone.put(drone.gimbal.set_pitch_and_yaw, 0, 0)\n")
drone.put(drone.gimbal.set_pitch_and_yaw, 0, 0)

time.sleep(2)
input("\ndrone.put(drone.gimbal.set_pitch_and_yaw, -90, 0)\n")
drone.put(drone.gimbal.set_pitch_and_yaw, -90, 0)

time.sleep(2)
input("\ndrone.put(drone.gimbal.set_pitch_and_yaw, 0, 0)\n")
drone.put(drone.gimbal.set_pitch_and_yaw, 0, 0)


time.sleep(2)
input("\ndrone.geofence.clear_geofence()\n")
drone.geofence.clear_geofence()

time.sleep(2)
input("\ndrone.put(drone.action.goto_location, 47, 8, 20, 0)\n")
drone.maneuver_to(10, 0, 0)

time.sleep(2)
input("\ndrone.land()\n")
drone.land()

time.sleep(2)
input("\ndrone.disarm()\n")
drone.disarm()

time.sleep(2)
input("\ndrone.wait_until_queue_empty()\n")
drone.wait_until_queue_empty()

time.sleep(2)
input("\ndrone.stop()\n")
drone.stop()
