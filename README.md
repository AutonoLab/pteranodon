# Pteranadon

<img src="https://static.wikia.nocookie.net/animals/images/a/a1/Pterathumb.png/revision/latest?cb=20200311123111" alt="drawing" width="200"/>

![Formatting](https://github.com/AutonoLab/pteranodon/actions/workflows/format-checks.yaml/badge.svg?branch=main)
![Unit Tests](https://github.com/AutonoLab/pteranodon/actions/workflows/unit-tests.yaml/badge.svg?branch=main)
![Integration Tests](https://github.com/AutonoLab/pteranodon/actions/workflows/integration-tests.yaml/badge.svg?branch=main)
![PyPI Build](https://github.com/AutonoLab/pteranodon/actions/workflows/build-check.yaml/badge.svg?branch=main)


A framework to built on top of MAVSDK which provides physical/virtual drone abstraction, abstraction for all async calls,
gives the end users an arduino-eqsue interface, and provides movement and utility methods based on relative coordinate systems.

---

## Requirements for Use
* python 3.8+
* PX4_Autopilot (either local or remote)
* mavsdk>=1.4.0
* numpy>=1.23.0
* pymavlink>=2.4.37
* grpcio>=1.47.0
* pyserial>=3.5
* Additional requirements needed for code in the implementations module

## Platform Compatibility

### Ubuntu (Most Supported)
   * **Supported for development and visual simulation**
   * Gazebo simulator (non-headless) requires Ubuntu 
   * PX4 has wider compatibility between Unix systems including Ubuntu

### Windows/WSL (Supported)
   * **Supported for development and visual simulation**
   * WSL Ubuntu 20.0.4 container
   * Gazebo simulator's window can be passed through
   * PX4 compatible

### macOS (Least Supported)
   * **Supported for development and *headless* simulation**
   * [PX4 Gazebo Headless](https://github.com/JonasVautherin/px4-gazebo-headless) Docker container allows for the running of the PX4 drone in the background.
   * Gazebo visualization in docker with X-server forwarding is possible, but not successfully reproduced yet.

## Project Installation

`pip install pteranodon`

### Installation from source

1. Download pteranodon source code using `git`:
   * `git clone https://github.com/Autonolab/pteranodon.git`
2. Run the `build` target from the Makefile:
   * `cd pteranodon`
   * `make build`

## Usage

For more information on the setup and usage of pteranodon, please refer to [USAGE.md](USAGE.md)

## Running Simulations

For more information on the usage of pteranodon in simulations environments, please refer to [SIMULATION.md](SIMULATION.md)

## Contributing

Welcome and thank you very much for your contribution. For the process of submitting PR, please refer to [CONTRIBUTING.md](CONTRIBUTING.md)。

### Contributors

This project exists thanks to all the people who contribute. [Contribute](CONTRIBUTING.md).

<a href="https://github.com/AutonoLab/pteranodon/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=AutonoLab/pteranodon"/>
</a>


      
