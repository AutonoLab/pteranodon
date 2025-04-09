#!/usr/bin/env python3
from __future__ import annotations

# setup uvloop for execution policuy
import asyncio
import uvloop
asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

# handle remainder of imports
import csv
import math
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

# Earth's radius in meters used for the conversion
EARTH_RADIUS = 6371000  

def read_mission(filepath, base_lat, base_lon, base_alt):
    # Read mission waypoints from a CSV file named "coordinates.txt"
    # Each line in the file should be: x, y, z
    mission_items = []
    print(f"Reading coordinates from {filepath}...")
    with open(filepath, "r") as f:
        reader = csv.reader(f)
        rows = [row for row in reader]
        for i, row in enumerate(rows):
            # Skip empty or malformed rows
            if not row or len(row) < 3:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                z = float(row[2])
            except ValueError:
                print("Skipping invalid line:", row)
                continue

            # Convert the relative coordinates to latitude and longitude offsets.
            # Note: Here x is assumed to be east, y is assumed to be north.
            delta_lat = (y / EARTH_RADIUS) * (180 / math.pi)
            delta_lon = (x / (EARTH_RADIUS * math.cos(math.radians(base_lat)))) * (180 / math.pi)

            # Compute global position by adding the offset to the home coordinates.
            lat = base_lat + delta_lat
            lon = base_lon + delta_lon
            alt = base_alt + z  # Altitude offset

            print(f"Waypoint from file: relative (x={x}, y={y}, z={z}) converted to global (lat={lat}, lon={lon}, alt={alt})")

            is_fly_through = i < (len(rows) - 1)

            # Create a mission item:
            # - speed: 5 m/s (change as needed)
            # - is_fly_through: True, so the drone continues smoothly through the waypoint.
            # - gimbal angles are set to NaN (not used in this case).
            # - No camera action.
            item = MissionItem(
                lat,
                lon,
                alt,
                speed_m_s=5,
                is_fly_through=is_fly_through,
                gimbal_pitch_deg=0,
                gimbal_yaw_deg=0,
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=0,
                camera_photo_interval_s=0,
                acceptance_radius_m=1.0,
                yaw_deg=0,
                camera_photo_distance_m=0,
                vehicle_action=MissionItem.VehicleAction.NONE
            )
            mission_items.append(item)

    return mission_items


async def print_position(drone):
    async for data in drone.telemetry.position_velocity_ned():
        print(data)
        asyncio.sleep(0.5)


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")
        

async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


async def run():
    # Connect to the drone via UDP (adjust the system address as needed)
    drone = System()
    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    # Wait for the drone to connect by monitoring the connection state.
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Wait for the home position from telemetry.
    # The home position is used as a reference to convert relative coordinates.
    home_position = None
    async for pos in drone.telemetry.home():
        home_position = pos
        break

    if home_position is None:
        print("ERROR: Home position not available. Exiting...")
        return

    base_lat = home_position.latitude_deg
    base_lon = home_position.longitude_deg
    base_alt = 0.0
    print(f"Home position: latitude={base_lat}, longitude={base_lon}, altitude={base_alt}")

    mission_items = read_mission("coordinates.txt", base_lat, base_lon, base_alt)
    if not mission_items:
        print("No valid mission items found. Exiting...")
        return
    
    running_tasks = []
    running_tasks.append(
        asyncio.ensure_future(print_mission_progress(drone))
    )
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks)
    )

    # Create the MissionPlan from the mission items.
    mission_plan = MissionPlan(mission_items)
    await drone.mission.set_return_to_launch_after_mission(True)

    # Upload the mission to the drone.
    print("Uploading mission...")
    await drone.mission.upload_mission(mission_plan)
    print("Mission uploaded.")

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # Arm the drone before starting the mission.
    print("Arming drone...")
    await drone.action.arm()

    # Start the mission.
    print("Starting mission...")
    await drone.mission.start_mission()

    # wait for mission to end
    await termination_task


if __name__ == "__main__":
    # Run the async main function.
    asyncio.run(run())
