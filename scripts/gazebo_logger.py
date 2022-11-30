import os
import time
import pandas as pd
import pyulog
from pyulog import ULog

def read_ulog(ulog_filename, messages=None):
    """
    Convert ulog to pandas dataframe.
    """
    log = pyulog.ULog(ulog_filename, messages)

    for msg in log.data_list:
        msg_data = pd.DataFrame.from_dict(msg.data)

    return log


from pteranodon import SimpleDrone

# DIR = "../gazebo_logfiles/"
DIR = "./"

print("Initiating drone...")

drone = SimpleDrone("udp://:14540")
entries = drone.log_files.get_entries()

if(len(entries) != 0):
    if not os.path.exists(DIR):
        os.makedirs(DIR)

for count in range(len(entries)):
    filename = "log_" + str(count)
    
    # while True:
    #     try:
    #         drone.log_files.download_log_file(entries[count], str(DIR + "RAW/" + filename + ".txt"))
    #         time.sleep(10)
    #         break
    #     except:
    #         time.sleep(15)
    
    print("drone.log_files.download_log_file(entries[" + str(count) + "], " + str(DIR + 'RAW/' + filename + '.txt') + ")")
    drone.log_files.download_log_file(entries[count], str(DIR + "RAW/" + filename + ".txt"))   
    
    os.system("ulog2csv -o " + DIR + "CSVs/" + filename + ".csv " + DIR + "RAW/" + filename + ".txt")

