#include <tuple>
#include <vector>
#include <iostream>
#include <limits>

#include "Eigen-3.3/Eigen/Dense"

#include "ego.hpp"
#include "json.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace Eigen;

using namespace std;

// Version 1:
// Evaluate the cost of each state.
// for KL: the cost should be high if a vehicle is close
// for PLCL: the cost should be high if a vehicle is in the lane
// for PLCR: the cost should be high if a vehicle is in the lane

vector<vector<double> >
Ego::getBestTrajectory() {

  EgoState next_state;
  vector<vector<double> > best_trajectory;
  double best_ref_vel;

  if (_state == EgoState::KL) {
    cerr << "In state KL in lane:" << _lane << " with s:" << _s << "speed " << _speed << endl;
    vector< tuple<EgoState, double, vector<vector<double> >, double > > state_costs;
    double lowest_cost = MAX_COST;
    bool found = false;


    double PLCL_speed = getSpeedClosestBehind(_lane - 1, found);
    if (!found) {
      PLCL_speed = _ref_vel;
    } else {
      cerr << "Setting PLCL_speed at " << PLCL_speed << endl;
    }

    double PLCR_speed = getSpeedClosestBehind(_lane + 1, found);
    if (!found) {
      PLCR_speed = _ref_vel;
    } else {
      cerr << "Setting PLCR_speed at " << PLCR_speed << endl;
    }

    /* TODO: to set a smoother acceleration and jerk. Currently generatePath
     * takes only a single _s and single _d. This should be changed to 5-th
     * order polynomial. We'll also have to change the _ref_vel accordingly.
     */
    state_costs.push_back(make_tuple(EgoState::KL, costKL(), _trajectory->generatePath(_s, _lane, _ref_vel), _ref_vel));
    state_costs.push_back(make_tuple(EgoState::KLA, costKLA(), _trajectory->generatePath(_s, _lane, _ref_vel), _ref_vel));
    state_costs.push_back(make_tuple(EgoState::PLCL, costPLCL(), _trajectory->generatePath(_s, _lane, PLCL_speed), PLCL_speed));
    state_costs.push_back(make_tuple(EgoState::PLCR, costPLCR(), _trajectory->generatePath(_s, _lane, PLCR_speed), PLCR_speed));


    // Iterate through cost options
    for (int i = 0; i < state_costs.size(); ++i ) {
      cerr << "found cost " << get<1>(state_costs[i]) << endl;
      if (lowest_cost > get<1>(state_costs[i])) {
        next_state = get<0>(state_costs[i]);
        lowest_cost = get<1>(state_costs[i]);
        best_trajectory = get<2>(state_costs[i]);
        best_ref_vel = get<3>(state_costs[i]);
      }
    }
  } else if (_state == EgoState::PLCL) {
    cerr << "In state PLCL" << endl;

    best_trajectory = _trajectory->generatePath(_s, _lane - 1, _ref_vel);
    _lane -= 1;
    best_ref_vel = _ref_vel;
    next_state = EgoState::KL;
  } else if (_state == EgoState::PLCR) {
    cerr << "In state PLCR" << endl;

    best_trajectory = _trajectory->generatePath(_s, _lane + 1, _ref_vel);
    _lane += 1;
    best_ref_vel = _ref_vel;
    next_state = EgoState::KL;
  } else if (_state == EgoState::KLA) {
    cerr << "In state KLA" << endl;

    best_trajectory = _trajectory->generatePath(_s, _lane, _ref_vel + 1);
    next_state = EgoState::KL;
    best_ref_vel = _ref_vel + 1;
  }


  _state = next_state;
  _ref_vel = best_ref_vel;
  return best_trajectory;
}

double
Ego::costKL() {
  double max_cost = exp(-30);

  for (auto& v : _vehicles) {
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

  cerr << "PLCL: consider lane change to: " << getLane(_d) - 1 << endl;
  if ((getLane(_d) - 1) < 0) {
    cerr << "PLCL: returning max cost" << endl;
    return MAX_COST;
  }

  for (auto& v : _vehicles) {
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

  cerr << "PLCR: consider lane change to: " << getLane(_d) + 1 << endl;
  // Ensure that the car doesn't leave the highway.
  if ((getLane(_d) + 1) > 3) {
    cerr << "PLCR: returning max cost" << endl;
    return MAX_COST;
  }

  for (auto& v : _vehicles) {
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

double
Ego::costKLA()
{
  if ((_speed + 1) > 49.5 || (_ref_vel + 1) > 49.5) {
    cerr << "costKLA found that we would reach over speed limit" << endl;
    return MAX_COST;
  }

  double max_cost = exp(-40);

  for (auto& v : _vehicles) {
    if (v.possible_collision(_s, _lane - 1, _speed + 1)) {
      // Increase cost for cars that are closer by.
      // TODO: add speed to cost estimation.
      double current_cost = exp(-(v.future_s() - _s));
      if (current_cost > max_cost) {
        max_cost = current_cost;
      }
    }
  }

  return max_cost;
}

double
Ego::getSpeedClosestBehind(int lane, bool& found_car) {
  double closest_distance = numeric_limits<double>::max();
  Vehicle* closest_car;
  found_car = false;

  for (auto& v : _vehicles) {
    if (v.in_lane(lane) && !v.in_front(_s) && v.in_range(_s, lane, _speed)) {
      double current_distance = _s - v.future_s();
      if (current_distance < closest_distance) {
        closest_distance = current_distance;
        closest_car = &v;
        found_car = true;
      }
    }
  }

  if (found_car) {
    cerr << "found closest car with speed " << closest_car->speed() << endl;
    return closest_car->speed();
  }

  return numeric_limits<double>::max();
}


void
Ego::set_sd_derivatives(vector<double>& old_d, vector<double>& old_s)
{
  _s_dot = old_s[2] - old_s[1];
  _s_dot_dot = old_s[2] - 2 * old_s[1] + old_s[0];

  _d_dot = old_d[2] - old_d[1];
  _d_dot_dot = old_d[2] - 2 * old_d[1] + old_d[0];

  cerr << "s_dot: " << _s_dot << " s_dot_dot:" << _s_dot_dot << " d_dot:" << _d_dot << " d_dot_dot:" << _d_dot_dot << endl;
}
