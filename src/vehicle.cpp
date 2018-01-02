#include <stdlib.h>
#include <math.h>

#include "utilities.hpp"
#include "vehicle.hpp"
#include "json.hpp"

#include <iostream>

Vehicle::Vehicle(nlohmann::basic_json<>::value_type& sensor_readings, int num_forward)
{
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
Vehicle::future_s()
{
  return _s + 0.02 * _speed * _num_forward;
}

double
Vehicle::get_lane()
{
  return getLane(_d);
}

bool
Vehicle::in_lane(int lane)
{
  return _d < (2 + 4 * lane + 2) && _d > (2 + 4 * lane - 2);
}

bool
Vehicle::in_front(double s)
{
  return future_s() > s;
}

bool
Vehicle::possible_collision(double s, int lane, double speed)
{
  double collision_distance = speed / MPH_TO_MS_CONSTANT * 4.0;
  return in_front(s) && in_range(s, lane, speed);
}

bool
Vehicle::in_range(double s, int lane, double speed) {
  // We want to detect at least 4 seconds at the speed we are as collision
  // detection
  double collision_distance = speed / MPH_TO_MS_CONSTANT * 4.0;
  return fabs(future_s() - s) < collision_distance && in_lane(lane);
}
