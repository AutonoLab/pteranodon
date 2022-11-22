import os
import time

from pteranodon import SimpleDrone

print("Initiating drone...")

drone = SimpleDrone("udp://:14540")

entries = drone.log_files.get_entries()


DIR = "../gazebo_logfiles/raw/"

if(len(entries) != 0):
    if not os.path.exists(DIR):
        os.makedirs(DIR)

count = 0

for entry in entries:
    drone.log_files.download_log_file(entry, str(DIR + str(count) + ".txt"))
    count = count + 1


    


