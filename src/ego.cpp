#include <tuple>
#include <vector>
#include <iostream>

#include "ego.hpp"
#include "json.hpp"

// Version 1:
// Evaluate the cost of each state.
// for KL: the cost should be high if a vehicle is close
// for PLCL: the cost should be high if a vehicle is in the lane
// for PLCR: the cost should be high if a vehicle is in the lane

using namespace std;

vector<vector<double> >
Ego::getBestTrajectory() {

  EgoState next_state;
  vector<vector<double> > best_trajectory;

  if (_state == EgoState::KL) {
    cerr << "In state KL in lane:" << _lane << " with s:" << _s << "speed " << _speed << endl;
    vector< tuple<EgoState, double, vector<vector<double> > > > state_costs;
    double lowest_cost = MAX_COST;

    state_costs.push_back(make_tuple(EgoState::KL, costKL(), _trajectory->generatePath(_s, _lane, _ref_vel)));
    state_costs.push_back(make_tuple(EgoState::PLCL, costPLCL(), _trajectory->generatePath(_s, _lane, _ref_vel)));
    state_costs.push_back(make_tuple(EgoState::PLCR, costPLCR(), _trajectory->generatePath(_s, _lane, _ref_vel)));


    // Iterate through cost options
    for (int i = 0; i < state_costs.size(); ++i ) {
      if (lowest_cost > get<1>(state_costs[i])) {
        next_state = get<0>(state_costs[i]);
        lowest_cost = get<1>(state_costs[i]);
        best_trajectory = get<2>(state_costs[i]);
      }
    }
    cerr << "found lowest cost " << lowest_cost << endl;
  } else if (_state == EgoState::PLCL) {
    cerr << "In state PLCL" << endl;

    best_trajectory = _trajectory->generatePath(_s, _lane - 1, _ref_vel);
    _lane -= 1;
    next_state = EgoState::KL;
  } else if (_state == EgoState::PLCR) {
    cerr << "In state PLCR" << endl;

    best_trajectory = _trajectory->generatePath(_s, _lane + 1, _ref_vel);
    _lane += 1;
    next_state = EgoState::KL;
  }

  _state = next_state;
  return best_trajectory;
}

double
Ego::costKL() {
  double max_cost = exp(-30);

  for (auto v : _vehicles) {
    if (v.possible_collision(_s, _lane, _speed)) {
      // Increase cost for cars that are closer by.
      // TODO: add speed to cost estimation.
      double current_cost = exp(-(v.future_s() - _s));
      if (current_cost > max_cost) {
        max_cost = current_cost;
      }
    }
  }

  cerr << "cost for KL in lane " << _lane << " with cost " << max_cost <<endl;

  return max_cost;
}

double
Ego::costPLCL() {
  double max_cost = exp(-30);

  if (_lane - 1 < 0) {
    return MAX_COST;
  }

  for (auto v : _vehicles) {
    if (v.possible_collision(_s, _lane - 1, _speed)) {
      // Increase cost for cars that are closer by.
      // TODO: add speed to cost estimation.
      double current_cost = exp(-(v.future_s() - _s));

      if (current_cost > max_cost) {
        max_cost = current_cost;
      }
    }
  }

  cerr << "cost for PLCL in lane " << _lane - 1 << " with cost " << max_cost * 2 << endl;
  return max_cost * 2.0;
}

double
Ego::costPLCR() {
  double max_cost = exp(-30);

  // Ensure that the car doesn't leave the highway.
  if (_lane + 1 > 3) {
    return MAX_COST;
  }

  for (auto v : _vehicles) {
    if (v.possible_collision(_s, _lane + 1, _speed)) {
      // Increase cost for cars that are closer by.
      // TODO: add speed to cost estimation.
      double current_cost = exp(-(v.future_s() - _s));
      if (current_cost > max_cost) {
        max_cost = current_cost;
      }
    }
  }

  cerr << "cost for PLCR in lane " << _lane + 1 << " with cost " << max_cost * 3. << endl;
  return max_cost * 3.0;
}
