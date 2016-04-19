# Cornell Ranger - Simulator

This is a repository for the simulation code (Matlab/C version) for Cornell Ranger. The resulting simulator moves the state forward by one time-step, automatically detecting and resolving ground contact.

## Model / Method:
- Planar (2D) dynamics
- Ground is an arbitrary smooth function
- Feet are convex, approximated by piecewise 5th-order splines that are second-order continuous (allows for use in trajectory optimization).
- Contact constraints are handled at the position level, and are satisfied at the end of each time-step to first-order accuracy.
- Joint constraints (ankles + hip) are handled using minimal coordinates.
- Equations of motion are computed analytically for each of four contact modes:
    - Flight (both feet in the air)
    - Double Stance (both feet on the ground)
    - Single Stance One (Foot one is rolling on the ground)
    - Single Stance Two (Foot two is rolling on the ground)
- Sliding contact is neglected. This simplifies the problem, and enables fast calculations. In rare cases this can cause small negative ground reaction forces. If you're curious about this, you might look up: [Infinite Friction Sliding](http://www.birs.ca/workshops/2014/14w5147/files/InfiniteFrictionSliding.pdf), "Wobbling, toppline, and forces of contact" by Tad McGeer (1989), or email me.


## Entry-point scripts:

### Derive_Dynamics.m
This script uses the Matlab symbolic toolbox to generate optimized matlab functions for computing low-level dynamics. All computer generated files are named autoGen_*.m.

### TEST_simulate.m
This script is just used to check that all of the elements of the simulation are working properly. It sets up a simulation, and then marches through time, drawing the robot as it goes.


## Important functions:

### simulate.m
This is the key function in this directory. It advances the state by one time-step. The inputs are the state vector, joint-torque vector, and parameter struct. The outputs are the next state, contact locations, contact forces, and boolean vector for which feet have an active contact.

### goldenSectionSearch.m
This function is actually not used directly. It is a simplified version of Matlab's fminbnd algorithm, adapted so that it can be compiled to either C or Mex. A modified version is used inside of findContactPoint.m to compute the minimum distance between each foot and the ground model. Accurate solutions for the contact point location allow for smooth rolling of the feet - an important feature for accurate calculation of energy use.

### ground model.m
The ground model user here is a linear + sine function, with a total of five parameters. In later versions, I hope to include additional sine terms to make more complex ground shapes.

## Mex and C-source

All of the simulation code is designed to work well with Matlab coder. Once you have mex properly configured, you should be able to run the command: `coder -build simulate.prj` to compile a mex version of the simulation function for your computer architecture.

To create a C-source or executable version, run: `coder simulate.prj` and edit the build target accordingly.

If you are interested in the C-code version of this simulator, and do not have Matlab coder, just send me an email and I'll send you a recently compiled version.