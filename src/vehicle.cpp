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
Vehicle::possible_collision(double s, int lane) {
  return in_range(s, lane, 15, 5);
}

bool
Vehicle::in_range(double s, int lane, double dist_in_front, double dist_in_back) {
  if (in_lane(lane)) {
    double closest_to_us_back = fmin(fmax(future_s() - s, 0), fmax(_s - s, 0));
    double closest_to_us_front = fmin(fmax(s - future_s(), 0), fmax(s - _s, 0));
    cerr << "in_range found lane to be accurate " << endl;
    if (closest_to_us_front > 0) {
      cerr << "closest_to_us_front positive: " << closest_to_us_front <<  endl;
      return closest_to_us_front < dist_in_front;
    } else {
      cerr << "closest_to_us_front equal to zero, closest_to_us_back: " << closest_to_us_back << endl;
      return closest_to_us_back < dist_in_back;
    }
  }
  return false;
}

bool
Vehicle::in_range_front(double s, int lane, double range) {
  return in_front(s) && ((future_s() - s) < range) && in_lane(lane);
}
