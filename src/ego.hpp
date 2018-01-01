#ifndef _EGO_HPP_
#define _EGO_HPP_

#include <vector>

#include "json.hpp"
#include "vehicle.hpp"


enum EgoState {
  KL, // Keep lane constant speed
  KLD, // Keep lane decrease speed
  KLA, // Keep lane increase speed
  PLCL, // Plan lane change left
  PLCR // Plane lane change right
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
  EgoState _state;
  std::vector<Vehicle> _vehicles;


public:
  Ego(double x, double y, double s, double d, double yaw, double speed, int lane, EgoState state) :
    _x(x),
    _y(y),
    _s(s),
    _d(d),
    _yaw(yaw),
    _speed(speed),
    _lane(lane),
    _state(state)
  {
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
  EgoState state() { return this->_state; };
};

#endif
