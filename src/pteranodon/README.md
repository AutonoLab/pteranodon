# Drone

## Plugins

`AbstractDrone` and it's subclasses use a FIFO queue for plugin methods. This allows these methods to be executed
in the order they are called. To allow for a delay in processing between commands in the queue, a time slice
can be set either in the initializer or after the fact. 

All plugin based commands that are called directly through the `AbstractDrone` object are added directly to the queue.
Methods are wrapped and pushed to the queue inside `AbstractDrone` for commands which under typical circumstances should
be run in a specific order. All other methods can be added to the queue or called directly.

These methods include:

### Base Plugins

**Action**
- `Action.arm` &#8594; `AbstractDrone.arm` 
- `Action.disarm` &#8594; `AbstractDrone.disarm`
- `Action.do_orbit` &#8594; `AbstractDrone.do_orbit`
- `Action.goto_location` &#8594; `AbstractDrone.goto_location`
- `Action.hold` &#8594; `AbstractDrone.hold` 
- `Action.kill` &#8594; `AbstractDrone.kill`
- `Action.land` &#8594; `AbstractDrone.land` 
- `Action.reboot` &#8594; `AbstractDrone.reboot`
- `Action.return_to_launch` &#8594; `AbstractDrone.return_to_launch` 
- `Action.set_actuator` &#8594; `AbstractDrone.set_actuator`
- `Action.set_current_speed` &#8594; `AbstractDrone.set_current_speed`
- `Action.set_maximum_speed` &#8594; `AbstractDrone.set_maximum_speed`
- `Action.set_return_to_launch_altitude` &#8594; `AbstractDrone.set_return_to_launch_altitude`
- `Action.set_takeoff_altitude` &#8594; `AbstractDrone.set_takeoff_altitude`
- `Action.shutdown` &#8594; `AbstractDrone.shutdown`
- `Action.takeoff` &#8594; `AbstractDrone.takeoff`
- `Action.terminate` &#8594; `AbstractDrone.terminate`
- `Action.transition_to_fixedwing` &#8594; `AbstractDrone.transition_to_fixedwing`
- `Action.transition_to_multicopter` &#8594; `AbstractDrone.transition_to_multicopter`

**Offboard**
- `Offboard.set_acceleration_ned` &#8594; `AbstractDrone.set_acceleration_ned`
- `Offboard.set_actuator_control` &#8594; `AbstractDrone.set_actuator_control`
- `Offboard.set_attitude` &#8594; `AbstractDrone.set_attitude`
- `Offboard.set_attitude_rate` &#8594; `AbstractDrone.set_attitude_rate`
- `Offboard.set_position_global` &#8594; `AbstractDrone.set_position_global`
- `Offboard.set_position_ned` &#8594; `AbstractDrone.set_position_ned`
- `Offboard.set_position_velocity_ned` &#8594; `AbstractDrone.set_position_velocity_ned`
- `Offboard.set_velocity_body` &#8594; `AbstractDrone.set_velocity_body`
- `Offboard.set_velocity_ned` &#8594; `AbstractDrone.set_velocity_ned`
- `Offboard.offboard_hold` &#8594; `AbstractDrone.offboard_hold`

### Extension Plugins
- `Relative.maneuver_to` &#8594; `AbstractDrone.maneuver_to`
- `Relative.create_geofence` &#8594; `AbstractDrone.create_geofence`

Other plugin methods can be manually added to the queue using the `AbstractDrone.put` method. Methods can be directly
passed in as the `Callable` type for simple control, or can be wrapped in the `AbstractDrone.Command` dataclass for more
complex control.

The `AbstractDrone.Command` dataclass allows for the optional addition of many needed features such as,
- Setting a priority of this command. By default, all commands have a priority of 1 with lower priority commands being run last.
Commands are sorted by priority when a command has been executed or when a command is added to the queue via the `put` command.
- The option to preempt the scheduling of the queue and add this command to the front of the queue, setting it to be next for execution.
If `preempt` is `True`, the priority is overridden to be equal to 1. 
- The ability to set a "handler" which is executed, not when the command has finished, but instead when it has been processed
from the queue.