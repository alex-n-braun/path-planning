#include "trajectory.h"

Trajectory::Trajectory(const dvector & s, const dvector & x, const dvector & y)
{
  spline_x.set_points(s, x);
  spline_y.set_points(s, y);
}

dvector Trajectory::operator()(double s)
{
  return {spline_x(s), spline_y(s)};
}

