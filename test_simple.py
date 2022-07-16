from simple import Simple

drone = Simple()

try:
    drone.arm()
except Exception as e:
    print(e)

drone.stop()
