#include "trajectory.hpp"
#include "utilities.hpp"
#include "ego.hpp"

#include "spline.h"
#include "json.hpp"

#include <vector>
#include <stdlib.h>

Trajectory::Trajectory(Ego* ego, nlohmann::basic_json<>::value_type* prev_path_x,
      nlohmann::basic_json<>::value_type* prev_path_y, double end_path_s, double end_path_d,
    vector<double>* map_waypoints_s, vector<double>* map_waypoints_x, vector<double>* map_waypoints_y) :
  _ego(ego),
  _prev_path_x(prev_path_x),
  _prev_path_y(prev_path_y),
  _end_path_s(end_path_s),
  _end_path_d(end_path_d),
  _map_waypoints_s(map_waypoints_s),
  _map_waypoints_x(map_waypoints_x),
  _map_waypoints_y(map_waypoints_y)
{
  int prev_size = _prev_path_x->size();
  _ref_x = ego->x();
  _ref_y = ego->y();
  _ref_yaw = deg2rad(ego->yaw());

  if (prev_size < 2) {
    double prev_car_x = _ref_x - cos(ego->yaw());
    double prev_car_y = _ref_y - sin(ego->yaw());

    // Use two points to make the path tangent to the previous path's end point.
    _ptsx.push_back(prev_car_x);
    _ptsx.push_back(_ref_x);

    _ptsy.push_back(prev_car_y);
    _ptsy.push_back(_ref_y);
  } else {
    _ref_x = (*_prev_path_x)[prev_size - 1];
    _ref_y = (*_prev_path_y)[prev_size - 1];

    double ref_x_prev = (*prev_path_x)[prev_size - 2];
    double ref_y_prev = (*prev_path_y)[prev_size - 2];
    _ref_yaw = atan2(_ref_y - ref_y_prev, _ref_x - ref_x_prev);

    _ptsx.push_back(ref_x_prev);
    _ptsx.push_back(_ref_x);

    _ptsy.push_back(ref_y_prev);
    _ptsy.push_back(_ref_y);
  }
}

vector<vector<double> >
Trajectory::generatePath(double car_s, int lane, double ref_vel)
{
  // First copy existing waypoints so that we can use it in the future.
  vector<double> ptsx(_ptsx);
  vector<double> ptsy(_ptsy);

  // Calculate our waypoints.
  vector<double> next_wp0 = getXY(car_s + 30.0, 2 + 4 * lane, *_map_waypoints_s, *_map_waypoints_x, *_map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60.0, 2 + 4 * lane, *_map_waypoints_s, *_map_waypoints_x, *_map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90.0, 2 + 4 * lane, *_map_waypoints_s, *_map_waypoints_x, *_map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Shift, rotate the global coordinates to local coordinates.
  for (int i = 0; i < ptsx.size(); ++i) {
    double shift_x = ptsx[i] - _ref_x;
    double shift_y = ptsy[i] - _ref_y;

    ptsx[i] = shift_x * cos(-_ref_yaw) - shift_y * sin(-_ref_yaw);
    ptsy[i] = shift_x * sin(-_ref_yaw) + shift_y * cos(-_ref_yaw);
  }

  tk::spline spline;
  spline.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for (int i = 0; i < _prev_path_x->size(); ++i) {
    next_x_vals.push_back((*_prev_path_x)[i]);
    next_y_vals.push_back((*_prev_path_y)[i]);
  }

  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0.0;

  for (int i = 0; i <= 50 - _prev_path_x->size(); ++i) {
    double N = target_dist / (0.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = spline(x_point);

    x_add_on = x_point;

    double x_point_glob = x_point * cos(_ref_yaw) - y_point * sin(_ref_yaw) + _ref_x;
    double y_point_glob = x_point * sin(_ref_yaw) + y_point * cos(_ref_yaw) + _ref_y;

    next_x_vals.push_back(x_point_glob);
    next_y_vals.push_back(y_point_glob);
  }

  return {next_x_vals, next_y_vals};
}

