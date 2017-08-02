#include "trajectory.h"

TrajectorySpline::TrajectorySpline()
{

}

TrajectorySpline::TrajectorySpline(const dvector & time, const dvector & x, const dvector & y)
{
  spline_x.set_points(time, x);
  spline_y.set_points(time, y);
}

dvector TrajectorySpline::operator()(double time) const
{
  return {spline_x(time), spline_y(time)};
}

dvector TrajectorySpline::tangent(double time) const
{
  return {spline_x.derivative(time), spline_y.derivative(time)};
}

dvector TrajectorySpline::normal(double time) const
{
  return {spline_y.derivative(time), -spline_x.derivative(time)};
}

dvector TrajectorySpline::oriented_curvature(double time) const // se definition of curvature on wikipedia
{
  dvector d1(tangent(time));
  dvector d2({spline_x.derivative2(time), spline_y.derivative2(time)});
  // matrix A be given by d1 and d2 as columns; then the determinant of A is
  double det(d1[0]*d2[1]-d1[1]*d2[0]);

  /*
  // length of tangent
  double len(lenvec(d1));
  dvector n({d1[1], -d1[0]}); n = scalevec(n, 1./len); // unit normal vector

  return scalevec(n, det/(len*len*len));

  This can be done better; we save one sqrt this way:
  */

  dvector n({d1[1], -d1[0]}); // normal vector, length is not necessarily one.
  double len2(sprod(n,n));
  return scalevec(n, det/len2*len2);
}

double TrajectorySpline::curvature(double time) const // se definition of curvature on wikipedia
{
  dvector d1(tangent(time));
  dvector d2({spline_x.derivative2(time), spline_y.derivative2(time)});
  // matrix A be given by d1 and d2 as columns; then the determinant of A is
  double det(d1[0]*d2[1]-d1[1]*d2[0]);

  // length of tangent
  double len(lenvec(d1));

  return det/(len*len*len);
}

