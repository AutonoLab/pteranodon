import os
from datetime import datetime
import sys
import pandas as pd
import pyulog
from pyulog import ULog


""" python3 ./gazebo_logger.py [entry of individual index to download] """

def read_ulog(ulog_filename, messages=None):
    """
    Convert ulog to pandas dataframe.
    """
    log = pyulog.ULog(ulog_filename, messages)

    for msg in log.data_list:
        msg_data = pd.DataFrame.from_dict(msg.data)

    return log


from pteranodon import SimpleDrone

#Output directory. Contains date so logs aren't overwritten
DIR = "../gazebo_logfiles-" + str(datetime.now().year) + "-" + str(datetime.now().month) + "-" + str(datetime.now().day) + "--" + str(datetime.now().hour) + ":" + str(datetime.now().minute) + "/"

print("Initiating drone...")

drone = SimpleDrone("udp://:14540")


entries = drone.log_files.get_entries()

print(sys.argv)

if(len(sys.argv) == 2):
    entries = entries[int(sys.argv[0])]
elif(len(sys.argv) > 2):
    sys.exit("ERROR: Too many arguments, expected <= 1")


#If entries present
if(len(entries) != 0):
    if not os.path.exists(DIR + "RAW/"):
        os.makedirs(DIR + "RAW/")
    if not os.path.exists(DIR + "CSVs/"):
        os.makedirs(DIR + "CSVs/")

#Download and convert all logs to CSV
for count in range(len(entries)):
    filename = "log_" + str(count)
    
    #Download
    drone.log_files.download_log_file(entries[count], str(DIR + "RAW/" + filename + ".txt"), 330)   
    
    #Convert to CSV
    try:
        os.system("ulog2csv -o " + DIR + "CSVs/" + filename + ".csv " + DIR + "RAW/" + filename + ".txt")
    except Exception as exception:
        drone.logger.error(exception)