import time

from gazebo import Gazebo

drone = Gazebo()

drone.arm()
drone.takeoff()

time.sleep(5)

drone.land()
time.sleep(1)
drone.stop()
