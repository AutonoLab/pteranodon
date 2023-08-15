from pteranodon import SimpleDrone
# from pteranodon.plugins.meta_plugins.ros import Ros
import time
import sys


if __name__ == '__main__':
    drone = SimpleDrone("udp://:14540")
    # if drone._plugins._test_valid_plugin_name('ros'):
    drone._plugins.meta_plugins['ros'].start()
    print("Start ROS")
    time.sleep(20)
    drone._plugins.meta_plugins['ros'].stop()
    print("Stop ROS")
    sys.exit()
