import os
import time

from pteranodon import SimpleDrone

DIR = "../gazebo_logfiles/raw/"

print("Initiating drone...")

drone = SimpleDrone("udp://:14540")
entries = drone.log_files.get_entries()

if(len(entries) != 0):
    if not os.path.exists(DIR):
        os.makedirs(DIR)

for count in range(len(entries)):
    drone.log_files.download_log_file(entries[count], str(DIR + str(count) + ".txt"), 5.0)
