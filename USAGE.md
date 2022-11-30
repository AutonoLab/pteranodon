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


### Multi Vehicle Simulation (Optional)

When using the PX4 Toolchain, multiple vehicles can be simulated in a world by running the below command.

`Tools/gazebo_sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>] [-w <world>] [-s <script>] [-t <target>]`

For example:

`Tools/gazebo_sitl_multiple_run.sh -s typhoon_h480:1,iris:1 -w baylands_park`

Will spawn one `typhoon_h480` and one `iris` drone in the `baylands_park` world. 

[PX4 Multi-Spawn Documentation](http://docs.px4.io/main/en/simulation/multi_vehicle_simulation_gazebo.html)

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
>>> drone.disarm()                                             # Disarm the drone
>>> drone.stop()                                               # Stop the drone and end program execution
```

### Custom Drone Objects

TEST