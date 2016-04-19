# Cornell Ranger <br/> Simulation and Trajectory Optimization

This repository contains some of the base code that I used for my PhD work with the [Cornell Ranger](http://ruina.tam.cornell.edu/research/topics/locomotion_and_robotics/ranger/Ranger2011/) walking robot. Key Features:
- Full derivation of the minimal coordinates dynamics
- Trajectory optimization
- Full derivation of floating base dynamics
- General-purpose time-stepping simulator for Ranger

### RangerSimulator

This directory contains a time-stepping simulator for Ranger, which allows for arbitrary sequencing of contact configurations and walking over interesting ground profiles.

The entire simulator is compiled to MEX for faster run-times. The C-code is automatically generated using the Matlab coder, making it easy to compile.

In addition to the simulator, this directory contains all of the code to derive the floating base dynamics using the Matlab symbolic toolbox.

### TrajectoryOptimization

This directory contains code to perform trajectory optimization over the minimal-coordinates model of Ranger. The trajectory optimization is currently performed using [GPOPS-II](http://www.gpops2.com/), although it would not be much work to convert it to my own open-source Matlab trajectory optimization: [TrajOpt](https://github.com/MatthewPeterKelly/TrajOpt).

I've also included all of the code to derive the equations of motion for Ranger in minimal-coordinates, again using the Matlab symbolic toolbox.

The code in this directory is incomplete -- it contains many of the pieces required for creating a full trajectory tracking controller, but they've not yet been assembled.

### Extras

This directory contains several small projects that I worked on that are related to Ranger. They are less-documented then the rest of the work, and are included here for the curious reader, rather than as a finished project.
