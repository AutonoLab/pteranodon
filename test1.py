import pteranodon
from mavsdk.mission import MissionItem, MissionPlan
import math


def wait_for_enter(message):
    input(f"{message} (Press Enter to continue...)")

def main():
    # Center point of the circle
    center_lat = 47.398039859999997
    center_lon = 8.5455725400000002
    radius = 0.0005  # approximately 50 meters
    num_points = 8  # number of points to create the circle

    mission_items = []
    
    # Create circular waypoints
    for i in range(num_points):
        angle = (2 * math.pi * i) / num_points
        lat = center_lat + radius * math.cos(angle)
        lon = center_lon + radius * math.sin(angle)
        
        # Make the last point non-fly-through
        is_fly_through = i < (num_points - 1)
        
        mission_items.append(
            MissionItem(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_altitude_m=10,
                speed_m_s=5,
                is_fly_through=is_fly_through,
                gimbal_pitch_deg=0,
                gimbal_yaw_deg=0,
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=0,
                camera_photo_interval_s=0,
                acceptance_radius_m=5,
                yaw_deg=0,
                camera_photo_distance_m=0,
                vehicle_action=MissionItem.VehicleAction.NONE
            )
        )

    mission_plan = MissionPlan(mission_items)

    print("Initializing drone...")
    drone = pteranodon.SimpleDrone("udp://:14540")
    wait_for_enter("Drone initialized")

    print("Uploading mission...")
    drone.mission.upload_mission(mission_plan)
    wait_for_enter("Mission uploaded")

    print("Arming drone...")
    drone.arm()
    wait_for_enter("Drone armed")

    print("Taking off...")
    drone.takeoff()
    wait_for_enter("Drone has taken off")

    print("Starting mission...")
    drone.mission.start_mission()
    wait_for_enter("Mission completed")

    print("Landing drone...")
    drone.land()
    wait_for_enter("Drone has landed")

    print("Disarming drone...")
    drone.disarm()
    wait_for_enter("Drone disarmed")


if __name__ == "__main__":
    main()

