#ifndef _TRAJECTORY_HPP_
#define _TRAJECTORY_HPP_

#include <vector>

#include "json.hpp"

class Ego;

using namespace std;

class Trajectory {
private:
  Ego* _ego;
  nlohmann::basic_json<>::value_type* _prev_path_x;
  nlohmann::basic_json<>::value_type* _prev_path_y;
  double _end_path_s;
  double _end_path_d;
  vector<double> _ptsx;
  vector<double> _ptsy;
  double _ref_x;
  double _ref_y;
  double _ref_yaw;
  vector<double>* _map_waypoints_s;
  vector<double>* _map_waypoints_x;
  vector<double>* _map_waypoints_y;

public:
  Trajectory(Ego* ego, nlohmann::basic_json<>::value_type* prev_path_x,
      nlohmann::basic_json<>::value_type* prev_path_y, double end_path_s, double end_path_d,
      vector<double>* map_waypoints_s, vector<double>* map_waypoints_x,
      vector<double>* map_waypoints_y);

  ~Trajectory() {};

  vector<vector<double> > generatePath(double car_s, int lane, double ref_vel);
};

#endif
