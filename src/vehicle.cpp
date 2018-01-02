#include <stdlib.h>
#include <math.h>

#include "utilities.hpp"
#include "vehicle.hpp"
#include "json.hpp"

#include <iostream>

Vehicle::Vehicle(nlohmann::basic_json<>::value_type& sensor_readings, int num_forward)
{
  this->_id = sensor_readings[0];
  this->_x = sensor_readings[1];
  this->_y = sensor_readings[2];
  this->_vx = sensor_readings[3];
  this->_vy = sensor_readings[4];
  this->_s = sensor_readings[5];
  this->_d = sensor_readings[6];
  this->_speed = sqrt(this->_vx * this->_vx + this->_vy + this->_vy);
  this->_num_forward = num_forward;
}

double
Vehicle::future_s()
{
  return this->_s + 0.02 * this->_speed * this->_num_forward;
}

double
Vehicle::get_lane()
{
  return getLane(_d);
}

bool
Vehicle::in_lane(int lane)
{
  return this->_d < (2 + 4 * lane + 2) && this->_d > (2 + 4 * lane - 2);
}

bool
Vehicle::in_front(double s)
{
  return this->future_s() > s;
}

bool
Vehicle::possible_collision(double s, int lane, double speed)
{
  // We want to detect at least 10 seconds at the speed we are as collision
  // detection
  double collision_distance = speed / MPH_TO_MS_CONSTANT * 4.0;
  return this->in_front(s) && (this->future_s() - s) < collision_distance && this->in_lane(lane);
}

