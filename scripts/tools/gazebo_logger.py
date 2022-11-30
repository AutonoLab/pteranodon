import os
from datetime import datetime
import time
import sys


""" python3 ./gazebo_logger.py [entry of individual index to download] """

from pteranodon import SimpleDrone

#Output directory. Contains date so logs aren't overwritten
DIR = "../gazebo_logfiles/" + str(datetime.now().year) + "-" + str(datetime.now().month) + "-" + str(datetime.now().day) + "--" + str(datetime.now().hour) + "-" + str(datetime.now().minute) + "/"


print("Initiating drone...")
drone = SimpleDrone("udp://:14540")

entries = drone.log_files.get_entries()

#If entries present
if(len(entries) != 0):
    if not os.path.exists(DIR + "RAW/"):
        os.makedirs(DIR + "RAW/")
    if not os.path.exists(DIR + "CSVs/"):
        os.makedirs(DIR + "CSVs/")



if(len(sys.argv) == 1):
    #Download and convert all logs to CSV
    for count in range(len(entries)):
        filename = "log_" + str(count)
        
        #Download
        drone.log_files.download_log_file(entries[count], str(DIR + "RAW/" + filename + ".txt"), 1000)   
        
        #Wait
        while(drone.log_files.get_download_progress().progress != 1.0):
            time.sleep(5)

        #Convert to CSV
        os.system("ulog2csv -o " + DIR + "CSVs/" + filename + ".csv " + DIR + "RAW/" + filename + ".txt")


elif(len(sys.argv) == 2):
    entry = entries[int(sys.argv[1])]

    filename = "log_" + str(sys.argv[1])
        
    #Download
    drone.log_files.download_log_file(entry, str(DIR + "RAW/" + filename + ".txt"), 1000)   

    #Wait
    while(drone.log_files.get_download_progress().progress != 1.0):
        time.sleep(5)
    
    #Convert to CSV
    os.system("ulog2csv -o " + DIR + "CSVs/" + filename + ".csv " + DIR + "RAW/" + filename + ".txt")


else:
    sys.exit("ERROR: Too many arguments, expected <= 1")

drone.wait_until_queue_empty()
drone.stop()
