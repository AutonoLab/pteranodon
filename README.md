# Pteranadon

<img src="https://static.wikia.nocookie.net/animals/images/a/a1/Pterathumb.png/revision/latest?cb=20200311123111" alt="drawing" width="200"/>

![Black Formatting Badge](https://github.com/AutonoLab/pteranodon/actions/workflows/black-check.yaml/badge.svg?branch=main)
![Flake8 Badge](https://github.com/AutonoLab/pteranodon/actions/workflows/flake8-check.yaml/badge.svg?branch=main)
![MyPy Badge](https://github.com/AutonoLab/pteranodon/actions/workflows/mypy-check.yaml/badge.svg?branch=main)
![PyLint Badge](https://github.com/AutonoLab/pteranodon/actions/workflows/pylint-check.yaml/badge.svg?branch=main)
![Unit Tests](https://github.com/AutonoLab/pteranodon/actions/workflows/unit-tests.yaml/badge.svg?branch=main)
![Integration Tests](https://github.com/AutonoLab/pteranodon/actions/workflows/integration-tests.yaml/badge.svg?branch=main)

## Formatting

Pteranodon uses the [PEP-8 Coding Standard](https://peps.python.org/pep-0008/) with required typing through
[MyPy](https://mypy.readthedocs.io/en/stable/).

In order to merge into the `main` branch, a contributor must satisfy these standards as well as the
[Black Code Style](https://black.readthedocs.io/en/stable/).

This can be done automatically by setting your IDE to format on save, or you can run `black --safe ./pteranodon` locally.

Description\
A framework to built on top of MAVSDK which provides physical/virtual drone abstraction, abstraction for all async calls,
gives the end users an arduino-eqsue interface, and provides movement and utility methods based on relative coordinate systems.
---

## Requirements for Use
* mavsdk>=1.4.0
* numpy
* grpcio
* PX4_Autopilot
* Linux* (Gazebo only runs on linux, but can use WSL)

## Project Installation
This project can be installed using WSL and Docker. Look below for instructions on both
### WSL Method
#### Intall WSL2
To install WSL2 with the default Ubuntu distribution on a new installation of Windows 11:
1. Open cmd.exe as administrator. This can be done by pressing the start key, typing cmd, right klicking on the Command prompt entry and selecting Run as administrator.
2. Execute the command wsl --install to run the installation routine for WSL.
3. Reboot the computer.
4. Open cmd again as a normal user (not as administrator). This can be done by pressing the Start key, typing cmd and pressing Enter.
5. Execute the command wsl to access the WSL shell.
6. WSL will prompt you for a user name and password for the Ubuntu installation. Record these credentials as you will need them later on!

#### Opening a WSL Shell
All operations to install and build PX4 must be done within a WSL Shell (you can use the same shell that was used to install WSL2 or open a new one).
1. Open a command prompt:
   * Press the Windows Start key.
   * Type cmd and press Enter to open the prompt.
4. To start WSL and access the WSL shell, execute the command:
   * `wsl`

Note: Enter the following commands to first close the WSL shell, and then shut down WSL:
* `exit`
* `wsl --shutdown`

Alternatively, after entering exit you can just close the prompt

#### Install PX4 Toolchain

Next we download the PX4 source code within the WSL2 environment, and use the normal Ubuntu installer script to to set up the developer environment. This will install the toolchain for Gazebo simulation, JMAVSim simulation and Pixhawk/NuttX hardware.

To install the development toolchain:
1. Open a WSL2 Shell (if it is still open you can use the same one that was used to install WSL2).
2. Execute the command `cd ~` to switch to the home folder of WSL for the next steps.

Warning: This is important! If you work from a location outside of the WSL file system you'll run into issues such as very slow execution and access right/permission errors.

1. Download PX4 source code using `git` (which is already installed in WSL2):
   * `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`
2. Run the ubuntu.sh installer script and acknowledge any prompts as the script progresses:
   * `bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`

Note: This installs tools to build PX4 for Pixhawk, Gazebo and JMAVSim targets:
* You can use the `--no-nuttx` and `--no-sim-tools` options to omit the NuttX and/or simulation tools.
* Other Linux build targets are untested (you can try these by entering the appropriate commands in Ubuntu Development Environment into the WSL shell).

1. Restart the "WSL computer" after the script completes (exit the shell, shutdown WSL, and restart WSL):
   * `exit`
   * `wsl --shutdown`
   * `wsl`
2. Switch to the PX4 repository in the WSL home folder:
   * `cd ~/PX4-Autopilot`
3. Build the PX4 SITL target and test your environment:
   * `make px4_sitl`

### Using Docker
waiting on steps from David

### Simulation Setup
* `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`
* `bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`

### Multi Vehicle Simulation
1. Tools/gazebo_sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>] [-w <world>]
2. Tools/gazebo_sitl_multiple_run.sh -s typhoon_h480:1,iris:1 -w baylands_park
      