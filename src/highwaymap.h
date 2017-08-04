#ifndef HIGHWAYMAP_H
#define HIGHWAYMAP_H

#include <vector>
#include <fstream>
#include "helpers.h"
#include "spline.h"
#include "trajectory.h"

//using namespace std;

struct Waypoint {
  double x, y, s, dx, dy;
  Waypoint(double x_, double y_, double s_, double dx_, double dy_)
    : x(x_), y(y_), s(s_), dx(dx_), dy(dy_) {}
};

class HighwayMap
{
public:
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
//  tk::spline map_splines_x;
//  tk::spline map_splines_y;
  Trajectory::Spline map_trajectory;
  // The max s value before wrapping around the track back to 0
  static const double max_s;
  HighwayMap();
  HighwayMap(std::ifstream & in_map);

  double distance(double x1, double y1, double x2, double y2) const;
  static double distance_s(double s1, double s2);
  int ClosestWaypoint(double x, double y) const;
  int NextWaypoint(double x, double y, double theta) const;
  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  dvector getFrenet(double x, double y, double theta) const;
  // Smoothened transform from Cartesian x,y coordinates to Frenet s,d coordinates;
  // see implementation for documentation
  dvector getSmoothFrenet(double x, double y) const;
  dvector getSmoothFrenet(const dvector & xy) const;
  // Transform from Frenet s,d coordinates to Cartesian x,y
  // (linear interplation between waypoints)
  dvector getXY(double s, double d) const;
  // Transform from Frenet s,d coordinates to Cartesian x,y
  // (spline interpolation)
  dvector getSmoothXY(double s, double d) const;
  dvector getSmoothXY(const dvector & sd) const;
  // get tangent on the street
  dvector tangent(double s) const;
  // get curvature of the street
  double curvature(double s) const;

  Waypoint operator[](int i) const;
};

#endif // HIGHWAYMAP_H
