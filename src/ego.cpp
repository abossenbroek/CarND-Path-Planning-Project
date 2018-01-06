#include <tuple>
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>


#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include "JMT.hpp"

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

EgoState
Ego::transition(const vector<double>& lane_speeds)
{
  double lowest_cost = MAX_COST;
  EgoState next_state = EgoState::KL;
  vector< tuple<EgoState, double> >
    state_costs = {
      make_tuple(EgoState::KL, costKL()),
      make_tuple(EgoState::KLA, costKLA(lane_speeds)),
      make_tuple(EgoState::KLD, costKLD()),
      make_tuple(EgoState::PLCL, costPLCL(lane_speeds)),
      make_tuple(EgoState::PLCR, costPLCR(lane_speeds)),
    };

  // Iterate through cost options
  for (int i = 0; i < state_costs.size(); ++i ) {
    cerr << "transition: for state: " << get<0>(state_costs[i]) << " got cost: " << get<1>(state_costs[i]) << endl;
    if (lowest_cost > get<1>(state_costs[i])) {
      next_state = get<0>(state_costs[i]);
      lowest_cost = get<1>(state_costs[i]);
    }
  }

  return next_state;
}


vector<vector<double> >
Ego::getBestTrajectory() {
  EgoState next_state = EgoState::KL;
  // Get the speed for each lane
  vector<double> lane_speeds = {0, 0, 0};
  get_lane_speed(lane_speeds);

  // Get the best next state if we have a state that permits this. This reduces
  // redundant calculations.
  EgoState candidate_state = transition(lane_speeds);
  vector<vector<double> > best_trajectory;
  double best_ref_vel = _ref_vel;
  vector<double> s_start = {_s, _s_dot, _s_dot_dot};
  vector<double> d_start = {_d, _d_dot, _d_dot_dot};

  switch (_state) {
    case EgoState::KLA:
      cerr << "### In state KLA" << endl;
      next_state = candidate_state;
      best_ref_vel = getMaxPermChangeRefSpeed(_ref_vel, lane_speeds[_lane]);
      best_trajectory = _trajectory->generatePath(_s, _lane, best_ref_vel );
      break;

    case EgoState::KLD:
      cerr << "### In state KLD" << endl;
      next_state = candidate_state;
      best_ref_vel = getMaxPermChangeRefSpeed(_ref_vel, lane_speeds[_lane]);
      best_trajectory = _trajectory->generatePath(_s, _lane, best_ref_vel);
      break;

    case EgoState::KL:
      cerr << "### In state KL" << endl;
      next_state =  candidate_state;
      best_trajectory = _trajectory->generatePath(_s, _lane, _ref_vel);
      best_ref_vel = _ref_vel;
      break;

    case EgoState::PLCL:
      cerr << "### In state PLCL" << endl;
      best_trajectory = _trajectory->generatePath(_s, _lane - 1, _ref_vel);
      _lane -= 1;
      best_ref_vel = _ref_vel;
      next_state = EgoState::KL;
      break;

    case EgoState::PLCR:
      cerr << "### In state PLCR" << endl;
      best_trajectory = _trajectory->generatePath(_s, _lane + 1, _ref_vel);
      _lane += 1;
      best_ref_vel = _ref_vel;
      next_state = EgoState::KL;
      break;

    default:
      cerr << "FSM State unknown";
  }

  _state = next_state;
  _ref_vel = best_ref_vel;
  return best_trajectory;
}

double
Ego::costKL() {
  double max_cost = exp(-5.05);

  for (auto& v : _vehicles) {
    if (v.in_range_front(_s, getLane(_d), 2)) {
      cerr << "KL: future_s:" << v.future_s() << endl;
      cerr << "KL: lane:" << v.get_lane() << endl;
      // Decrease cost for cars that are closer by since it should make this
      // option more interesting.
      return MAX_COST;
    }
  }

  return max_cost;
}

double
Ego::costKLA(const vector<double>& lane_speeds)
{
  if ((_speed + 1) > 49.5 || (_ref_vel + 1) > 49.5) {
    return MAX_COST;
  }

  double max_cost = exp(-5.1);

  for (auto& v : _vehicles) {
    if (v.in_range_front(_s, getLane(_d), 5)) {
      return MAX_COST;
    }
  }

  return max_cost;
}

double
Ego::costKLD()
{
  if ((_speed - 0.5) < 0 || (_ref_vel - 0.5) < 0) {
    cerr << "costKLD found that we would reach stand still" << endl;
    return MAX_COST;
  }

  double max_cost = MAX_COST;

  for (auto& v : _vehicles) {
    bool is_car_in_lane = v.in_lane(getLane(_d)) || v.in_lane(_lane);
    bool is_car_ahead = v.s() > _s;

    if (is_car_in_lane && is_car_ahead) {
      cerr << "KL: found vehicle at: " << v.s();
      double dist = v.s() - _s;
      // Add the second to ensure that KLD is more attractive only if lane
      // changes result in less than 1.2 mph increases.
      return exp(-5) * exp(-1.2);
    }
  }

  return max_cost;
}


double
Ego::costPLCL(const vector<double>& lane_speeds) {
  double max_cost = exp(-5);

  if (getLane(_d) != _lane) {
    cerr << "PLCL: found incomplete turn" << endl;
    return MAX_COST;
  }
  if ((getLane(_d) - 1) < 0 || (_lane - 1) < 0) {
    cerr << "PLCL: found unallowed turn" << endl;
    return MAX_COST;
  }

  for (auto& v : _vehicles) {
    if (v.possible_collision(_s, _lane - 1)) {
      cerr << "PLCL: found car at: " << v.future_s() << endl;

      return MAX_COST;
    }
  }

  cerr << "PLCL: will discount with " << fmin(1, exp(-( lane_speeds[_lane - 1] - lane_speeds[_lane]))) << endl;
  max_cost *= fmin(1, exp(-( lane_speeds[_lane - 1] - lane_speeds[_lane])));

  return max_cost;
}

double
Ego::costPLCR(const vector<double>& lane_speeds) {
  double max_cost = exp(-5);

  // Ensure that the car doesn't leave the highway.
  if ((getLane(_d) + 1) > 2 || (_lane + 1) > 2) {
    cerr << "PLCR: found unallowed turn" << endl;
    return MAX_COST;
  }

  if (getLane(_d) != _lane) {
    cerr << "PLCR: found incomplete turn" << endl;
    return MAX_COST;
  }

  for (auto& v : _vehicles) {
    if (v.possible_collision(_s, _lane + 1)) {
      cerr << "PLCR: found car at: " << v.future_s() << endl;
      return MAX_COST;
    }
  }
  cerr << "PLCR: will discount with " << fmin(1, exp(-( lane_speeds[_lane + 1] - lane_speeds[_lane]))) << endl;
  max_cost *= fmin(1, exp(-(lane_speeds[_lane + 1] - lane_speeds[_lane])));

  return max_cost;
}


  void
Ego::set_sd_derivatives(vector<double> d_prev, vector<double> s_prev, const vector<int>& steps)
{
  vector<double> time_steps = {0, 0.2 * max(steps[0], 1), 0.2 * (max(steps[0], 1) + max(steps[1], 1))};

  reverse(d_prev.begin(), d_prev.end());
  reverse(s_prev.begin(), s_prev.end());

  tk::spline spline_d;
  tk::spline spline_s;

  if (!(d_prev[0] < d_prev[1])) {
    d_prev[1] += 0.001;
  }
  if (!(d_prev[1] < d_prev[2])) {
    d_prev[2] += 0.001;
  }

  if (!(s_prev[0] < s_prev[1])) {
    s_prev[1] += 0.001;
  }
  if (!(s_prev[1] < s_prev[2])) {
    s_prev[2] += 0.001;
  }


  spline_d.set_points(time_steps, d_prev);
  spline_s.set_points(time_steps, s_prev);

  const double h = 1;

  // Center difference for better approximation
  _s_dot = (spline_s(2 * h) - spline_s(0)) / (2 * h);
  _s_dot_dot = (spline_s(2 * h) - 2 * spline_s(h) + spline_s(0)) / (h * h);

  // Center difference for better approximation
  _d_dot = (spline_d(2 * h) - spline_d(0)) / (2 * h);
  _d_dot_dot = (spline_d(2 * h) - 2 * spline_d(h) + spline_d(0)) / (h * h);
}

void
Ego::get_lane_speed(vector<double>& lane_speeds) {
  vector<int> cars_in_lane = {0, 0, 0};
  lane_speeds = {49.5, 49.5, 49.5};

  for (auto& v : _vehicles) {
    int lane = v.get_lane();

    if (lane >= 0 && lane <= 3 && v.in_front(_s) && (v.future_s() - _s) < 40) {
      lane_speeds[lane] = ((lane_speeds[lane] * cars_in_lane[lane]) +
          v.speed()) / (cars_in_lane[lane] + 1);
      cars_in_lane[lane]++;
    }
  }
}

double
Ego::getMaxPermChangeRefSpeed(double current_speed, double ref_speed)
{
  double to_return;
  // Calculate our maximum permitted acceleration.
  double max_perm_accel = fmin(20, fabs(ref_speed - speed()) / 0.02);
//  cerr << "getMaxPerm: max accel" << max_perm_accel << endl;
  double max_perm_speed_change = 0.02 * max_perm_accel;
 // cerr << "getMaxPerm: max_speed_change" << max_perm_speed_change << endl;

  if (current_speed >= ref_speed) {
    // We are going to fast so we need to slow down
    to_return = fmax(ref_speed - current_speed, -max_perm_speed_change);
  } else {
    // We can go faster so lets speed up
    to_return = fmin(ref_speed - current_speed, max_perm_speed_change);
  }

  to_return += current_speed;
  //cerr << "getMaxPerm: final speed" << max_perm_speed_change << endl;

  return to_return;
}

