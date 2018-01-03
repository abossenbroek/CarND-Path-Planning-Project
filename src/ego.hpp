#ifndef _EGO_HPP_
#define _EGO_HPP_

#include <vector>
#include <memory>

#include "json.hpp"
#include "vehicle.hpp"
#include "trajectory.hpp"
#include "utilities.hpp"

#define MAX_COST 1

// TODO: consider double left and double right lane change.

enum EgoState {
  KL, // Keep lane constant speed
  KLA, // Keep lane increase speed
  KLD, // Keep lane decrease speed
  PLCL, // Plan lane change left
  PLCR
};

class Ego {
private:
  double _x;
  double _y;
  double _s;
  double _d;
  double _yaw;
  double _speed;
  int _lane;
  double _ref_vel;
  EgoState _state;
  std::vector<Vehicle> _vehicles;
  shared_ptr<Trajectory> _trajectory;
  double _s_dot;
  double _s_dot_dot;
  double _d_dot;
  double _d_dot_dot;

  double costKL();
  double costKLA();
  double costKLD();
  double costPLCL(const vector<double>& lane_speeds);
  double costPLCR(const vector<double>& lane_speeds);

public:
  Ego(double x, double y, double s, double d, double yaw, double speed,
      int lane, double ref_vel, EgoState state,
      nlohmann::basic_json<>::value_type* prev_path_x,
      nlohmann::basic_json<>::value_type* prev_path_y,
      double end_path_s, double end_path_d,
      vector<double>* map_waypoints_s,
      vector<double>* map_waypoints_x,
      vector<double>* map_waypoints_y,
      vector<double>& d_prev,
      vector<double>& s_prev,
      int steps_prev) :
    _x(x),
    _y(y),
    _s(s),
    _d(d),
    _yaw(yaw),
    _speed(speed),
    _lane(lane),
    _ref_vel(ref_vel),
    _state(state)
  {
    _trajectory = make_shared<Trajectory>(Trajectory(this, prev_path_x, prev_path_y, end_path_s, end_path_d, map_waypoints_s, map_waypoints_x, map_waypoints_y));
    int s_prev_size = prev_path_x->size();
    if (s_prev_size > 0) {
      _s = end_path_s;
    }

    cerr << "********************************************************************************" << endl;
    cerr << "ref_vel:    " << ref_vel << endl;
    cerr << "speed:      " << speed << endl;
    cerr << "_s:         " << _s << endl;
    cerr << "lane:       " << lane << endl;
    cerr << "getLane:    " << getLane(d) << endl;
    cerr << "_d:         " << _d << endl;
    vector<int> time_steps = {50 - s_prev_size, steps_prev};

    set_sd_derivatives(d_prev, s_prev, time_steps);
  }

  ~Ego() {};

  void addVehicles(nlohmann::basic_json<>::value_type& sensor_fusion, int num_forward)
  {
    for (int i = 0; i < sensor_fusion.size(); ++i) {
      this->_vehicles.push_back(Vehicle(sensor_fusion[i], num_forward));
    }
  }

  double x() { return this->_x; };
  double y() { return this->_y; };
  double s() { return this->_s; };
  double d() { return this->_d; };
  double yaw() { return this->_yaw; };
  double speed() { return this->_speed; };
  double lane() { return this->_lane; };
  double ref_vel() { return this->_ref_vel; };
  EgoState state() { return this->_state; };
  Trajectory getTrajectory() { return *(this->_trajectory); };
  vector<vector<double> > getBestTrajectory();

  double getSpeedClosestBehind(int lane, bool& found_car);
  void set_sd_derivatives(vector<double> d_prev, vector<double> s_prev, const vector<int>& steps);


  void get_lane_speed(vector<double>& lane_speeds);
};

#endif
