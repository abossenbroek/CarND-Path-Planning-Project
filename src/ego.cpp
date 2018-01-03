#include <tuple>
#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>


#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

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

  EgoState next_state = EgoState::KL;
  vector<vector<double> > best_trajectory;
  double best_ref_vel = _ref_vel;
  vector<double> lane_speeds = {0, 0, 0};
  // Get the speed for each lane
  get_lane_speed(lane_speeds);

  if (_state == EgoState::KL) {
    cerr << "In state KL in lane:" << _lane << " with s:" << _s << "speed " << _speed << endl;
    vector< tuple<EgoState, double, vector<vector<double> >, double > > state_costs;
    double lowest_cost = MAX_COST + 0.1;

    /* TODO: to set a smoother acceleration and jerk. Currently generatePath
     * takes only a single _s and single _d. This should be changed to 5-th
     * order polynomial. We'll also have to change the _ref_vel accordingly.
     */
    state_costs.push_back(make_tuple(EgoState::KLD, costKLD(), _trajectory->generatePath(_s, _lane, _ref_vel), _ref_vel));
    state_costs.push_back(make_tuple(EgoState::KL, costKL(), _trajectory->generatePath(_s, _lane, _ref_vel), _ref_vel));
    state_costs.push_back(make_tuple(EgoState::KLA, costKLA(), _trajectory->generatePath(_s, _lane, _ref_vel), _ref_vel));
    state_costs.push_back(make_tuple(EgoState::PLCL, costPLCL(lane_speeds), _trajectory->generatePath(_s, _lane, _ref_vel), _ref_vel));
    state_costs.push_back(make_tuple(EgoState::PLCR, costPLCR(lane_speeds), _trajectory->generatePath(_s, _lane, _ref_vel), _ref_vel));


    // Iterate through cost options
    for (int i = 0; i < state_costs.size(); ++i ) {
      cerr << "for state: " << get<0>(state_costs[i]) << " found cost " << get<1>(state_costs[i]) << endl;
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

    best_ref_vel = getMaxPermChangeRefSpeed(_ref_vel, lane_speeds[_lane]);
    best_trajectory = _trajectory->generatePath(_s, _lane, best_ref_vel );
    next_state = EgoState::KL;
  } else if (_state == EgoState::KLD) {
    cerr << "In state KLD" << endl;

    best_ref_vel = getMaxPermChangeRefSpeed(_ref_vel, lane_speeds[_lane]);
    best_trajectory = _trajectory->generatePath(_s, _lane, best_ref_vel);
    next_state = EgoState::KL;
  }

  _state = next_state;
  _ref_vel = best_ref_vel;
  return best_trajectory;
}

double
Ego::costKL() {
  double max_cost = exp(-5);

  for (auto& v : _vehicles) {
    if (v.possible_collision(_s, _lane, _speed)) {
      // Decrease cost for cars that are closer by since it should make this
      // option more interesting.
      // TODO: add speed to cost estimation.
      double current_cost = 1. - exp(getCollisionDistance(_speed) / (_s - v.future_s()));
      if (current_cost > max_cost) {
        cerr << "cost for KL, found future_s:" << v.future_s() << " with _s: " << _s <<endl;
        max_cost = current_cost;
      }
    }
  }

  cerr << "cost for KL in lane " << _lane << " with cost " << max_cost <<endl;

  return max_cost;
}

double
Ego::costPLCL(const vector<double>& lane_speeds) {
  double max_cost = exp(-5.1);

  cerr << "PLCL: consider lane change to: " << getLane(_d) - 1 << " given _d:" << _d << endl;
  if ((getLane(_d) - 1) < 0 || (_lane - 1) < 0) {
    cerr << "PLCL: returning max cost" << endl;
    return MAX_COST;
  }
  if (getLane(_d) != _lane) {
    cerr << "PLCL: returning max cost because lane changed is not finished" << endl;
    return MAX_COST;
  }

  for (auto& v : _vehicles) {
    if (v.in_range(_s, _lane - 1, _speed)) {
      // Increase cost for cars that are closer by.
      // TODO: add speed to cost estimation.
      //double current_cost = exp(-fabs(v.future_s() - _s));
      double new_lane_speed = lane_speeds[_lane - 1] - 0.25 * v.speed();
      double current_cost = 1. - exp(getCollisionDistance(_speed) / (-(lane_speeds[_lane] - new_lane_speed) / new_lane_speed * fabs(_s - v.future_s())));
      cerr << "cost for PLCL, found future_s:" << v.future_s()
        << " with _s: " << _s
        << " lane_speeds[_lane]" << lane_speeds[_lane]
        << " and lane_speeds[_lane - 1]: " << lane_speeds[_lane - 1] << endl;

      if (current_cost > max_cost) {
        max_cost = current_cost;
      }
    }
  }

  cerr << "cost for PLCL in lane " << _lane - 1 << " with cost " << max_cost << endl;
  return max_cost;
}

// TODO: add collision detection for cars behind when shifting lanes!
double
Ego::costPLCR(const vector<double>& lane_speeds) {
  double max_cost = exp(-5.3);

  cerr << "PLCR: consider lane change to: " << getLane(_d) + 1 << " given _d:" << _d << endl;
  // Ensure that the car doesn't leave the highway.
  if ((getLane(_d) + 1) > 2 || (_lane + 1) > 2) {
    cerr << "PLCR: returning max cost" << endl;
    return MAX_COST;
  }

  if (getLane(_d) != _lane) {
    cerr << "PLCR: returning max cost because lane changed is not finished" << endl;
    return MAX_COST;
  }

  for (auto& v : _vehicles) {
    if (v.in_range(_s, _lane + 1, _speed)) {
      // Increase cost for cars that are closer by.
      // TODO: add speed to cost estimation.
      //double current_cost = exp(-fabs(v.future_s() - _s));
      double new_lane_speed = lane_speeds[_lane + 1] - 0.25 * v.speed();
      double current_cost = 1. - exp(getCollisionDistance(_speed) / (-(lane_speeds[_lane] - new_lane_speed) / new_lane_speed * fabs(_s - v.future_s())));
      cerr << "cost for PLCR, found future_s:" << v.future_s()
        << " with _s: " << _s
        << " lane_speeds[_lane]" << lane_speeds[_lane]
        << " and lane_speeds[_lane + 1]: " << lane_speeds[_lane + 1] << endl;
      if (current_cost > max_cost) {
        max_cost = current_cost;
      }
    }
  }

  cerr << "cost for PLCR in lane " << _lane + 1 << " with cost " << max_cost << endl;
  return max_cost;
}

double
Ego::costKLA()
{
  if ((_speed + 1) > 49.5 || (_ref_vel + 1) > 49.5) {
    cerr << "costKLA found that we would reach over speed limit" << endl;
    return MAX_COST;
  }

  if (getLane(_d) != _lane) {
    cerr << "KLA: returning max cost because lane changed is not finished" << endl;
    return MAX_COST;
  }

  double max_cost = exp(-40);

  for (auto& v : _vehicles) {
    if (v.possible_collision(_s, _lane, _speed + 1)) {
      // Increase cost for cars that are closer by.
      // TODO: add speed to cost estimation.
      double current_cost = 1. - exp(getCollisionDistance(_speed) / (-0.99 * fabs(_s - v.future_s())));
      if (current_cost > max_cost) {
        max_cost = current_cost;
      }
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

  double max_cost = exp(-20);

  for (auto& v : _vehicles) {
    if (v.possible_collision(_s, _lane, _speed - 1)) {
      // Decrease cost for cars that are closer by since it should make this
      // option more interesting.
      // TODO: add speed to cost estimation.
      double current_cost = 1. - exp(getCollisionDistance(_speed) / (_s - v.future_s()));
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
Ego::set_sd_derivatives(vector<double> d_prev, vector<double> s_prev, const vector<int>& steps)
{
  vector<double> time_steps = {0, 0.2 * max(steps[0], 1), 0.2 * (max(steps[0], 1) + max(steps[1], 1))};

  reverse(d_prev.begin(), d_prev.end());
  reverse(s_prev.begin(), s_prev.end());

  tk::spline spline_d;
  tk::spline spline_s;

  cerr << "time_steps: " << time_steps[0] << ", " << time_steps[1] << ", " << time_steps[2] << endl;
  cerr << "d_prev: " << d_prev[0] << ", " << d_prev[1] << ", " << d_prev[2] << endl;

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

  cerr << "s_dot: " << _s_dot << " s_dot_dot:" << _s_dot_dot << " d_dot:" << _d_dot << " d_dot_dot:" << _d_dot_dot << endl;
}

void
Ego::get_lane_speed(vector<double>& lane_speeds) {
  vector<int> cars_in_lane = {0, 0, 0};
  lane_speeds = {49.5, 49.5, 49.5};

  for (auto& v : _vehicles) {
    int lane = v.get_lane();

    if (lane >= 0 && lane <= 3 && v.in_front(_s) && (v.future_s() - _s) < 50) {
      lane_speeds[lane] = ((lane_speeds[lane] * cars_in_lane[lane]) +
          v.speed()) / (cars_in_lane[lane] + 1);
      cars_in_lane[lane]++;
    }
  }
}
