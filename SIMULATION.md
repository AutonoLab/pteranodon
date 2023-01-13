# Simulation
   
## Running virtual drones with PX4

### Multi Vehicle Simulation (Optional)

When using the PX4 Toolchain, multiple vehicles can be simulated in a world by running the below command.

`Tools/gazebo_sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>] [-w <world>] [-s <script>] [-t <target>]`

For example:

`Tools/gazebo_sitl_multiple_run.sh -s typhoon_h480:1,iris:1 -w baylands_park`

Will spawn one `typhoon_h480` and one `iris` drone in the `baylands_park` world. 

[PX4 Multi-Spawn Documentation](http://docs.px4.io/main/en/simulation/multi_vehicle_simulation_gazebo.html)

### Using pteranodon run.sh

When using the pteranodon system, multiploe vehicles can be simulated in a world by running the below command.

`scripts/run.sh [<model>] [<script>] ... [<model>] [<script>] [<world>]`

For example:

`./scripts/run.sh iris examples/abstract_drones/random_move.py baylands`

Will spawn one `iris` in the `baylands_park` world and the `iris` will execute the main of the `random_move.py` example implementation.

This can be extended to include as many `model, script` pairs as the user wants.

When the user is done with their simulation the executing scripts and Gazebo simulation can be killed by pressing "q". 

### Using terminals to control a drone

When a drone is present in Gazebo it can also be controlled manually.

Often a Gazebo instance with a single drone is started with the following command: 

`make px4_sitl gazebo_*` where * is the corresponding model

For example:

`make px4_sitl gazebo_iris`

Would start a Gazebo simulation in a blank world with the iris drone. This is useful for debugging AbstractDrone implementations done with pteranodon.

Once a drone is loaded, open a terminal and launch python3. Once Python is launched simply invoke the implementation of choice. 

For example:

```bash
$ python3
>>> from pteranodon import SimpleDrone
>>> drone = CustomDrone("udp://:14540")
>>> drone.arm()                            # Arm the drone
>>> drone.takeoff()                        # Takeoff from the ground
>>> drone.custom_action()                  # the custom action to execute
>>> drone.land()                           # Required to disarm the drone
>>> drone.disarm()                         # Disarm the drone
>>> drone.stop()                           # Stop the drone and end program execution
```

### Using terminals to control multiple drones

Controlling multiple drones is identical to controlling a single drone when using terminals, except multiple terminals will have to be opened. 

The drones which are started by PX4 in Gazebo have ports which start at 14540 and increment upwards for each model in the world. 

For example: 

`./scripts/run.sh custom_model custom_drone.py iris straight_line.py baylands`

The custom_model drone will have port 14540 and the iris drone will have port 14541. Thus it is important to make sure the Python scripts being used also match the ports.

In this scenario these would be an example of possible code present in custom_drone.py and straight_line.py:

Terminal 1 (custom_drone.py)
```bash
$ python3
>>> from pteranodon import AbstractDrone
>>> class CustomDrone(AbstractDrone):
.
.
.
>>> drone = CustomDrone("udp://:14540")
>>> drone.arm()                            # Arm the drone
>>> drone.takeoff()                        # Takeoff from the ground
```

Terminal 2 (straight_line.py)
```bash
$ python3
>>> from pteranodon import AbstractDrone
>>> class StraightLine(AbstractDrone):
.
.
.
>>> drone = StraightLine("udp://:14541")
>>> drone.arm()                            # Arm the drone
>>> drone.takeoff()                        # Takeoff from the ground
```

### Executing primary loop on drones

All drones using the AbstractDrone base class will have methods called: `loop`, `start_loop`, `stop_loop`

These methods allow the drone to execute continous behavior in a background thread, i.e. performing adversary drone detection on a camera feed.
Using these methods to execute a custom control loop in an autonomous drone occurs as follows:

```bash
$ python3
>>> from pteranodon import AbstractDrone 
>>> class ControlLoop(AbstractDrone):
.
.
.
>>>     def loop(self):  # simple control flow with attributes assumed to contain valid data
>>>         if self._adversary_visible:
>>>             self.maneuver_to(self._adversary_location)
.
.
.
>>> drone = CustomDrone("udp://:14540")
>>> drone.arm()                            # Arm the drone
>>> drone.takeoff()                        # Takeoff from the ground
>>> drone.start_loop()                     # Begins execution of the custom control loop
.                                          # This will repeatedly execute the loop function defined above
.                                          
.
>>> drone.stop_loop()                      # Ceases execution of custom control loop
>>> drone.land()                           # Required to disarm the drone
>>> drone.disarm()                         # Disarm the drone
>>> drone.stop()                           # Stop the drone and end program execution
```
