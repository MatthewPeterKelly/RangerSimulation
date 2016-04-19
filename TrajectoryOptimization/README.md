# Cornell Ranger - Trajectory Optimization

This directory contains code to perform trajectory optimization over the minimal-coordinates model of Ranger. The trajectory optimization is currently performed using [GPOPS-II](http://www.gpops2.com/), although it would not be much work to convert it to my own open-source Matlab trajectory optimization: [TrajOpt](https://github.com/MatthewPeterKelly/TrajOpt).

This code is incomplete, but contains main of the important code fragments to make a full trajectory tracking controller. The full equations of motion in minimal coordinates are derived here, as well as optimal trajectories for single stance and double stance.

## Entry-point scripts:

#### `Derive_*.m`
Use the matlab symbolic toolbox to generate equations of motion in minimal coordinates.

### `TEST_*.m`
Test scripts for either the dynamics or trajectory optimization functions.
