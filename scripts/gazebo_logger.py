import os

from pteranodon import SimpleDrone

print("Initiating drone...")

drone = SimpleDrone("udp://:14540")

print("Waitiing for the drone to connect...")


if drone.core.connection_state() == state.is_connected:
    print("Drone CONNECTED")



entries = drone.log_files.get_entries()


DIR = "../gazebo_logfiles/raw/"

if(len(entries) != 0):
    if not os.path.exists(DIR):
        os.makedirs(DIR)

count = 0

for entry in entries:
    drone.log_files.download_log_file(entry, DIR + str(count))
    count = count + 1


    


