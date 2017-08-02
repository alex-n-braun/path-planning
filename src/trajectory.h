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
  TrajectorySpline(const dvector & time, const dvector & x, const dvector & y);
  dvector operator()(double time) const;
  dvector tangent(double time) const;
  dvector normal(double time) const;
  dvector oriented_curvature(double time) const;
  double curvature(double time) const;
};

#endif // TRAJECTORY_H
