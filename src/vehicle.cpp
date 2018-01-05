#include <stdlib.h>
#include <math.h>

#include "utilities.hpp"
#include "vehicle.hpp"
#include "json.hpp"

#include <iostream>

Vehicle::Vehicle(nlohmann::basic_json<>::value_type& sensor_readings,
    int num_forward) {
  _id = sensor_readings[0];
  _x = sensor_readings[1];
  _y = sensor_readings[2];
  _vx = sensor_readings[3];
  _vy = sensor_readings[4];
  _s = sensor_readings[5];
  _d = sensor_readings[6];
  _speed = sqrt(_vx * _vx + _vy + _vy);
  _num_forward = num_forward;
}

double
Vehicle::future_s() {
  return _s + 0.02 * _speed * _num_forward;
}

double
Vehicle::get_lane() {
  return getLane(_d);
}

bool
Vehicle::in_lane(int lane) {
  return _d < (2 + 4 * lane + 2) && _d > (2 + 4 * lane - 2);
}

bool
Vehicle::in_front(double s) {
  return future_s() > s;
}

bool
Vehicle::possible_collision(double s, int lane, double speed) {
  return in_range(s, lane, speed, 1.5, 1.00);
}

bool
Vehicle::in_range(double s, int lane, double speed, double time_in_front, double time_in_back) {
  return (fmax(future_s(), future_s() - s) < fmax(getCollisionDistance(speed, time_in_front), getCollisionDistance(_speed, time_in_front))
      || fmax(future_s(), s - future_s()) < getCollisionDistance(_speed, time_in_back)) && in_lane(lane);
}

bool
Vehicle::in_range_front(double s, int lane, double speed, double range) {
  return in_front(s) && (future_s() - s) < getCollisionDistance(speed, range);
}
