# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Introduction
The program main.cpp allows to generate a path for a car in a simulator. The path will seek to avoid collisions, change lanes when the current lane is too slow and avoid sudden acceleratoin, breaking and jerks.

Below follows an explanation of how the paths are generated.

_note_: we recommend to use the chrome extension [github mathjax](https://github.com/orsharir/github-mathjax) to read this documentation to render the math.

## Path planner
To plan the paths the path planning module uses a finite state machine with corresponding cost functions. It also calculates the positions of the cars around itself through sensor fusion and first order polynomial.

### Finite State Machine
A Finite State Machine (FSM) permits to decide on the actions that should be taken at any given moment. The states are as follows,

 * _KL_: keep lane, don't change anything to the path.
 * _KLA_: keep lane accelerate, increase the reference speed. 
 * _KLD_: keep lane decelerate, decrease the reference speed.
 * _PLCL_: Plan lane change left 
 * _PLCR_: Plan lane change right, this state only returns a cost lower than the maximum cost if the resulting path is still within the lane change range and no collisions would be created by the lane change. If the lane change is permitted, plan a change to the right.

The permitted changes are illustrated in the following picture 

![image of FSM](https://raw.githubusercontent.com/abossenbroek/CarND-Path-Planning-Project/master/img/path_FSM.PNG). 

The decision to change into a state is based on cost functions. We explain these below.

### Cost function for change of lanes


### Path generation
We generate a path by creating a spline along our longitudinal and latitudinal data points that we want to visit. We then proceed by making the path discrete by steps dependent on the reference speed. This speed is changed by the _KLA_ and _KLD_ states.

## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Outcome
As shown below the current implementation allows to drive at least 8.47 miles without incident 

![image of long run](https://raw.githubusercontent.com/abossenbroek/CarND-Path-Planning-Project/master/img/udacity_path_planning_8-47miles.png).
