/*
 * =====================================================================================
 *
 *       Filename:  vehicle.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  31/12/17 13:50:26
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include <math.h>

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
  return 0.25 * this->_d - 2;
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
Vehicle::possible_collision(double s, int lane)
{
  return this->in_front(s) && (this->future_s() - s) < COLLISION_DISTANCE && this->in_lane(lane);
}

