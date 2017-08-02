#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "spline.h"
#include "helpers.h"

class TrajectorySpline
{
public:
  tk::spline spline_x;
  tk::spline spline_y;
  TrajectorySpline();
  TrajectorySpline(const dvector & s, const dvector & x, const dvector & y);
  dvector operator()(double s) const;
  dvector tangent(double s) const;
  dvector normal(double s) const;
  dvector oriented_curvature(double s) const;
  double curvature(double s) const;
};

#endif // TRAJECTORY_H
