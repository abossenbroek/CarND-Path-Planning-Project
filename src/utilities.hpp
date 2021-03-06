#ifndef _UTILITIES_HPP_
#define _UTILITIES_HPP_

#include <vector>
#include <string>
#include <math.h>

#define MPH_TO_MS_CONSTANT 2.237


using namespace std;

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
string hasData(string s);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
int getLane(double d);
void push_circular(vector<double>& circular_buffer, double new_val);
double getCollisionDistance(double speed, double time = 2.0);

#endif
