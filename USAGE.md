# Usage
   

## Drone Server Setup

_**If you have a PX4/MAVSDK server instance running on an external device the below steps are not necessary.**_

### Install PX4 Toolchain (Ubuntu/WSL only)

Next we download the PX4 source code and use the normal Ubuntu installer script to set up the developer environment. This will install the toolchain for Gazebo simulation, JMAVSim simulation and Pixhawk/NuttX hardware.

To install the development toolchain:
1. Open your shell environment (`bash`, `zsh`, WSL shell)
2. Run the `build-all` target from the Makefile:
   * `cd ~/path/to/pteranodon`
   * `make build-all`

### Run PX4 Docker Container (All platforms, required for macOS testing)

If you do not need a visual simulation or are running on macOS, you must run this docker container in the background to test your code.

`docker run -it jonasvautherin/px4-gazebo-headless:latest`

## Basic Drone Control

_This section assumes that you already have a drone server running, either through a [local simulation](#Drone-Server-Setup) 
or on your drone remotely via serial, TCP, or UDP_

In this example the server is running on `localhost` and the default UDP port of 14540.

### REPL Control

One of the best features of pteranodon is the ability to easily control your drone via the Python REPL. Rather than
swarming yourself with many different asyncio calls, each drone can be controlled very directly with the execution of some simple commands.

The example below does the bare minimum, starts a drone, arms it, has it take off, then land. 

```bash
$ python3
>>> from pteranodon import SimpleDrone
>>> drone = SimpleDrone("udp://:14540")
>>> drone.arm()                            # Arm the drone
>>> drone.takeoff()                        # Takeoff from the ground
>>> drone.land()                           # Required to disarm the drone
>>> drone.disarm()                         # Disarm the drone
>>> drone.stop()                           # Stop the drone and end program execution
```

This example is more complex. While also performing the same steps as the above example (required), this chain of commands 
moves the drone in a square. The `maneuver_to` command moves the drone in relative coordinates.

```bash
$ python3
>>> from pteranodon import SimpleDrone
>>> drone = SimpleDrone("udp://:14540")
>>> drone.arm()                            # Arm the drone
>>> drone.takeoff()                        # Takeoff from the ground
>>> drone.maneuver_to(10, 0, 0)            # Move forward 10
>>> drone.maneuver_to(0, 10, 0)            # Move to the right 10
>>> drone.maneuver_to(-10, 0, 0)           # Move back 10
>>> drone.maneuver_to(0, -10, 0)           # Move to the left 10 (ending in the start position)
>>> drone.land()                           # Required to disarm the drone
>>> drone.disarm()                         # Disarm the drone
>>> drone.stop()                           # Stop the drone and end program execution
```

This final example, while appearing more complex, actually executes the exact same code in the same order and the same method as the previous example.
However, visually, there are two differences:
1. `SimpleDrone` does not have an address as an argument.
   - By default, if no address is passed into the `AbstractDrone` class (which `SimpleDrone` implements), it will use a
   utility named `ServerDetector` which will automatically look for MAVSDK servers hosted on your local serial devices,
   as well as TCP and UDP ports (within a range) located on a specific IP address (by default `localhost`). `AbstractDrone` will choose
   one of these discovered addresses to connect to if found.
   - This behavior can be configured or disabled in the `AbstractDrone` initializer when implementing for your own
   [custom drone](#Custom-Drone-Objects).
   - The `ServerDetector` class can be used outside the `AbstractDrone` class. 
2. `drone.put(drone.relative.maneuver_to, ...)` replaces `drone.maneuver_to(...)`
   - Under the hood, this `put` call is being made when calling `maneuver_to` directly from the drone.
   - Any drone control method that can be directly accessed via the `drone` object is using this `put` call. 
   - The `put` method adds this specific method to an internal "command queue" to ensure that certain methods are
   executed in the order they are called. This is why all movement methods are wrapped in this way and directly accessible
   through the `drone` object. More information on the `put` method and the available calls are located in the
   [drone README](pteranodon/README.md).

```bash
$ python3
>>> from pteranodon import SimpleDrone
>>> drone = SimpleDrone()
>>> drone.arm()                                                # Arm the drone
>>> drone.takeoff()                                            # Takeoff from the ground
>>> drone.put(drone.relative.maneuver_to, 10, 0, 0)            # Move forward 10
>>> drone.put(drone.relative.maneuver_to, 0, 10, 0)            # Move to the right 10
>>> drone.put(drone.relative.maneuver_to, -10, 0, 0)           # Move back 10
>>> drone.put(drone.relative.maneuver_to, 0, -10, 0)           # Move to the left 10 (ending in the start position)
>>> drone.land()                                               # Required to disarm the drone
>>> drone.disarm()                                             # Disarm the drone
>>> drone.stop()                                               # Stop the drone and end program execution
```

### Custom Drone Objects

As previously mentioned, `AbstractDrone` is the required super-class of any custom drone implementations. 
In the previous section, we used the `SimpleDrone` class which is a basic implementation that does not provide any
extra functionality beyond what is present in `AbstractDrone`.

In this section we will explore how to accomplish more complex implementations.

```python
from pteranodon import AbstractDrone

class CustomDrone(AbstractDrone):
   
    def __init__(self, address: str):
       # Initialize the superclass, don't attempt auto-connection with empty address
        super().__init__(address=address, autoconnect_no_addr=False) 

    def setup(self):
        return

    def loop(self):
        return

    def teardown(self):
        return
```

The above implementation is not very different from the `SimpleDrone` implementation, but we can use it to illustrate the overall structure.

If you are familiar with the format of [Arduino](https://www.arduino.cc/reference/en/language/structure/sketch/loop/)
code, the functions inside this file may look similar. We'll break these down in this file, but they are detailed further
in the `AbstractDrone`'s method documentation.

1.  `setup()`
   - This is one of the first methods called and is called inside the `AbstractDrone` initializer. 
   - Because of this, it is better to put any plugin or sensor setup/initialization in this method rather than the initializer. 
2.  `loop()`
   - Each call of this function represents an iteration inside the drone's background "loop thread".
   - The loop thread is not started on initialization. You must call `self.start_loop()` from either inside or outside the
class to begin iterating the `loop()` function.
   - Because this function is called in a separate thread than main or the command threads, stalling inside this function
      will not cause commands to stop their execution. 
     - If this is the intended result and the command is being executed in the command thread, it is much better to call
         `self.wait(10.0, preempt=True)` to wait before executing the next command for 10 seconds.
   - If you want execution of this function to pause, you can pause the loop thread with `self.pause_loop()` and resume
      the thread with `self.resume_loop()`
     - _Note:_ If you call `self.stop_loop()` the loop thread will be stopped permanently.
3.  `teardown()`
   - This function is called when `AbstractDrone`'s `stop()` method is called. 
   - Use this method to perform any necessary teardown for custom plugins, sensors, or anything else before shutdown.


Now that the structure has been covered, we can create a fully custom implementation.

```python
from pteranodon import AbstractDrone
import time
from typing import List, Tuple

class CustomDrone(AbstractDrone):
   
    def __init__(self, address: str):
       # Initialize the superclass, don't attempt auto-connection with empty address
        super().__init__(address=address, autoconnect_no_addr=False) 
        
       # Move forward 10, right 10, back 10, to the left 10
        self._xy_dirs : List[Tuple[float, float]] = [(10, 0),
                                                     (0, 10),
                                                     (-10, 0),
                                                     (0, -10)]
        self._dir_idx = 0
        
        # You can start the loop automatically in the initializer or in your calls to this drone object.
        self.start_loop()

    def setup(self):
        self.arm()
        
        self.set_maximum_speed(20.0)
        self.set_current_speed(10.0)
        self.takeoff()
        return

    def loop(self):
        # Move in currently selected direction
        #   For example: self._dir_idx = 1, move forward 0, move to the right 10, move down 0.
        self.maneuver_to(*self._xy_dirs[self._dir_idx], 0)
        
        time.sleep(5.0) # Wait for the drone to finish moving
        
        # Cycle through direction index
        self._dir_idx += 1
        if self._dir_idx >= len(self._xy_dirs):
           self._dir_idx = 0
        return

    def teardown(self):
        self.return_to_launch()
        self.wait(5.0) # Wait for the drone to return to launch
        
        self.land()
        self.disarm()
        return
```

As you can see above, even without introducing custom plugins or sensors we can make our `CustomDrone` do many things.

At a high level, this drone (once instantiated) will take off and start moving at 10 m/s in a 10-meter square. 
Remember, since neither the commands nor the `loop` function are being executed on the main thread, your REPL or file
will still have complete access to the drone as it is executing these actions.

At this point, when the movement becomes boring, simply run `drone.stop()` and this drone will return to launch, land, and disarm.

_Note:_ Since all of these methods are places in the command queue, the `time.sleep` calls are not strictly necessary,
however, I added them to not `put` commands faster than the action could be performed, leading to some lost calls
(The queue has a max size of 10).

An example of this custom class being used in the REPL is below

```bash
$ python3
>>> from custom_drone import CustomDrone
>>> drone = CustomDrone("udp://:14540")
.
.
.
.
.
.
.
.
.
>>> drone.stop()
```
