#include "ego.hpp"
#include "json.hpp"

// Version 1:
// Evaluate the cost of each state.
// for KL: the cost should be high if a vehicle is close
// for PLCL: the cost should be high if a vehicle is in the lane
// for PLCR: the cost should be high if a vehicle is in the lane


//vector<vector<double> >
//Ego::getBestTrajectory(
