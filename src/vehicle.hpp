#ifndef _VEHICLE_HPP_
#define _VEHICLE_HPP_

#include <vector>
#include "json.hpp"

// Range within which a car in a similar lane will be considered to be in collision distance
#define COLLISION_DISTANCE 30

namespace path_planner {
class Vehicle {
  int _id;
  double _x;
  double _y;
  double _vx;
  double _vy;
  double _s;
  double _d;
  double _speed; // mph
  double _num_forward;


public:
  Vehicle(nlohmann::basic_json<>::value_type& sensor_readings, int num_forward);
  ~Vehicle() {};

  double future_s();
  double get_lane();
  bool in_lane(int lane);
  bool in_front(double s);
  bool possible_collision(double s, int lane, double speed);
  bool in_range(double s, int lane, double speed);

  int id() { return this->_id; };
  double x() { return this->_x; };
  double y() { return this->_y; };
  double vx() { return this->_vx; };
  double vy() { return this->_vy; };
  double s() { return this->_s; };
  double d() { return this->_d; };
  double speed() { return this->_speed; };
};

#endif // _VEHICLE_HPP_
