## traxter_bringup package

This package's purpose is to launch all other TIR-ANT software. There is no custom source code for this package, only launchers and configuration files.

Inside the [config](config/) folder you'll find:
- ```d435.json``` the configuration file loaded into the D435 RealSense camera when its wrapper is launched

- ```defaultHardware.yaml``` the general configuration file for the real robot

- ```defaultSimulation.yaml``` the general configuration file for the simulator environment

Inside the [launch](launch/) you'll find several launch files each with a specific task. The most general one which is tasked with launching all other launcher is [traxter_bringup.launch.py](launch/traxter_bringup.launch.py). The purpose of each launch file is explained in comment form on the actual file.