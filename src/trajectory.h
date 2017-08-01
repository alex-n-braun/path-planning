#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "spline.h"
#include "helpers.h"

class Trajectory
{
public:
  tk::spline spline_x;
  tk::spline spline_y;
  Trajectory(const dvector & s, const dvector & x, const dvector & y);
  dvector operator()(double s);
};

#endif // TRAJECTORY_H
