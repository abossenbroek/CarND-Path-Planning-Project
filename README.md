# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Introduction
The program main.cpp allows to generate a path for a car in a simulator. The path will seek to avoid collisions, change lanes when the current lane is too slow and avoid sudden acceleratoin, breaking and jerks.

Below follows an explanation of how the paths are generated.

_note_: we recommend to use the chrome extension [github mathjax](https://github.com/orsharir/github-mathjax) to read this documentation to render the math.

### Path generation
To generate the paths the path planning module uses a finite state machine with corresponding cost functions. It also calculates the positions of the cars around itself through sensor fusion and first order polynomial.

#### Finite State Machine
A Finite State Machine (FSM) permits to decide on the actions that should be taken at any given moment. The states are as follows,

 * _KL_: keep lane, don't change anything to the path.
 * _KLA_: keep lane accelerate, increase the reference speed by 0.5 mph
 * _KLD_: keep lane decelerate, decrease the reference speed by 0.5 mph
 * _PLCL_: Plan lane change left, this state only returns a cost lower than the maximum cost if the resulting path is still within the lane change range and no collisions would be created by the lane change. If the lane change is permitted, plan a change to the left.
 * _PLCR_: Plan lane change right, this state only returns a cost lower than the maximum cost if the resulting path is still within the lane change range and no collisions would be created by the lane change. If the lane change is permitted, plan a change to the right.

The permitted changes are illustrated in the following picture 

![image of FSM](https://raw.githubusercontent.com/abossenbroek/CarND-Path-Planning-Project/master/img/path_FSM.PNG). 

The decision to change into a state is based on cost functions. We explain these below.

#### Cost function for change of lanes
For the change of lanes we seek to capture sufficient dynamics that makes a lane change optimal in case that it allows us to reach a faster speed while avoiding collisions. To achieve this we define our cost as,
$$\arg\max_{c \in P(l\pm 1)}\Bigg\lbrace 1 - e^{\frac{s_C}{-\frac{v_{l^c} - (v_{l^{c\pm 1}} - 0.25 v_c)}{(v_{l^{c\pm 1}} - 0.25 v_c)}\lvert s_e - s_c\rvert }} \Bigg\rbrace,$$
where $c \in P(l\pm 1)$ is all the cars in the set of possible collision in lane plus or minus one. For all these cars we seek the maximum cost using a combination of the longitudinal data and lane speed data. As such, $s_C$ is the collision distance, $v_{l^c}$ is the velocity in the current lane, $v_{l^{c\pm 1}}$ is the velocity of a lane change to either left or right, $v_c$ is the velocity of the car that with which we could collide, $s_e$ the longitudinal position of the ego car and $s_c$ the future longitudinal position of the car with which we could collide. Note that if the car is not yet in its destination lane, which we know by inspecting the latitudinal data and the target lane, we return 1, which is the maximum cost. We do the same whenever the car seeks the cost to change into a non permitted lane.

#### Cost function of keeping lane
To estimate the cost of keeping the lane we only consider the cars in front of us. Given the set of cars with which we could collide we seek,
$$\arg\max_{c \in P(l)}\Bigg\lbrace 1 - e^{\frac{s_C}{-(s_e - s_c)}} \Bigg\rbrace,$$
this increases the cost for cars that are closer to us and decreases the cost for cars that further ahead.

#### Cost function changing 
To estimate the cost of keeping the lane we only consider the cars in front of us. Given the set of cars with which we could collide we seek,
$$\arg\max_{c \in P(l)}\Bigg\lbrace 1 - e^{\frac{s_C}{-K(s_e - s_c)}} \Bigg\rbrace,$$
where $K = 0.99$ for acceleration, and $K=1$ for deceleration. This will make it slightly cheaper to choose _KLD_ over _KLA_, which will cause our path planner to decrease speed.

#### Final remarks
The implementation of the $\arg\max$ is such that the initial cost if set to a lower initial value for the _KL_ state than all other states. This is to ensure that in the case of ties because no cars are in our lane we choose to stay in the current lane.

### Path generation
We generate a path by creating a spline along our longitudinal and latitudinal data points that we want to visit. We then proceed by making the path discrete by steps dependent on the reference speed. This speed is changed by the _KLA_ and _KLD_ states.

## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

## Outcome
As shown below the current implementation allows to drive at least 8.47 miles without incident 

![image of long run](https://raw.githubusercontent.com/abossenbroek/CarND-Path-Planning-Project/master/img/udacity_path_planning_8-47miles.png).
