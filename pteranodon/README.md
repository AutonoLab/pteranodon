# Drone

## Plugins

`AbstractDrone` and it's subclasses use a FIFO queue for plugin methods. This allows these methods to be executed
in the order they are called. To allow for a delay in processing between commands in the queue, a time slice
can be set either in the initializer or after the fact. 

All plugin based commands that are called directly through the `AbstractDrone` object are added directly to the queue. 

These methods include:

**Base Plugins**
- `Action.arm` &#8594; `AbstractDrone.arm` 
- `Action.disarm` &#8594; `AbstractDrone.disarm`
- `Action.hold` &#8594; `AbstractDrone.hold` 
- `Action.land` &#8594; `AbstractDrone.land` 
- `Action.return_to_launch` &#8594; `AbstractDrone.return_to_launch` 
- `Action.set_maximum_speed` &#8594; `AbstractDrone.set_maximum_speed`
- `Action.set_return_to_launch_altitude` &#8594; `AbstractDrone.set_return_to_launch_altitude`
- `Action.set_takeoff_altitude` &#8594; `AbstractDrone.set_takeoff_altitude`
- `Action.shutdown` &#8594; `AbstractDrone.shutdown`
- `Action.takeoff` &#8594; `AbstractDrone.takeoff`
- `Action.land` &#8594; `AbstractDrone.land`

**Extension Plugins**
- `Relative.maneuver_to` &#8594; `Relative.maneuver_to`
- `Relative.create_geofence` &#8594; `Relative.create_geofence`

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