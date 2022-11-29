# Usage
   

## Drone Server Setup

_**If you have a PX4/MAVSDK server instance running on an external device the below steps are not necessary.**_

### Install PX4 Toolchain (Ubuntu/WSL only)

Next we download the PX4 source code and use the normal Ubuntu installer script to set up the developer environment. This will install the toolchain for Gazebo simulation, JMAVSim simulation and Pixhawk/NuttX hardware.

To install the development toolchain:
1. Open your shell environment (`bash`, `zsh`, WSL shell)
2. Run the installation script.
   * `sudo ~/path/to/pteranodon/scripts/misc/install_px4.sh`

### Run PX4 Docker Container (All platforms, required for macOS testing)

If you do not need a visual simulation or are running on macOS, you must run this docker container in the background to test your code.

`docker run -it jonasvautherin/px4-gazebo-headless:latest`


### Multi Vehicle Simulation (Optional)

When using the Ubuntu/WSL PX4 Toolchain, multiple vehicles can be simulated in a world by running the below command.

`Tools/gazebo_sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>] [-w <world>] [-s <script>] [-t <target>] [-l <label>]`

For example:

`Tools/gazebo_sitl_multiple_run.sh -s typhoon_h480:1,iris:1 -w baylands_park`

Will spawn one `typhoon_h480` and one `iris` drone in the `baylands_park` world. 

[PX4 Multi-Spawn Documentation](http://docs.px4.io/main/en/simulation/multi_vehicle_simulation_gazebo.html)

