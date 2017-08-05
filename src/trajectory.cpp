#include <iostream>
#include "trajectory.h"
#include "Eigen-3.3/Eigen/Dense"
#include "highwaymap.h"


namespace Trajectory {

Spline::Spline()
{ }

Spline::Spline(const dvector & time, const dvector & x, const dvector & y)
{
  spline_x.set_points(time, x);
  spline_y.set_points(time, y);
}

Spline::Spline(const Spline &spline)
  : spline_x(spline.spline_x), spline_y(spline.spline_y)
{ }

dvector Spline::operator()(double time) const
{
  return {spline_x(time), spline_y(time)};
}

dvector Spline::tangent(double time) const
{
  return {spline_x.derivative(time), spline_y.derivative(time)};
}

dvector Spline::normal(double time) const
{
  return {spline_y.derivative(time), -spline_x.derivative(time)};
}

dvector Spline::oriented_curvature(double time) const // see definition of curvature on wikipedia
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

double Spline::curvature(double time) const // see definition of curvature on wikipedia
{
  dvector d1(tangent(time));
  dvector d2({spline_x.derivative2(time), spline_y.derivative2(time)});
  // matrix A be given by d1 and d2 as columns; then the determinant of A is
  double det(d1[0]*d2[1]-d1[1]*d2[0]);

  // length of tangent
  double len(lenvec(d1));

  return det/(len*len*len);
}

/* MinJerk Trajectory
 * jerk minimizing polynomials for s and d
 */

void MinJerk::solve(dvector &coeff, const Eigen::MatrixXd &A, const Eigen::MatrixXd &v)
{
  Eigen::MatrixXd Ai(A.inverse());
  Eigen::MatrixXd c(Ai * v);
  for (int i(0); i<3; i++)
    coeff[i+3] = c.data()[i];
}

void MinJerk::solve0(dvector & coeff, const TimeRange &time_span, const VecRange &range)
{
  Eigen::MatrixXd A(3,3);
  double dt(time_span.second-time_span.first);
  double dt2(dt*dt);
  double dt3(dt2*dt);
  double dt4(dt2*dt2);
  double dt5(dt2*dt3);
  A << dt3,        dt4,     dt5,
       3.*dt2,  4.*dt3,  5.*dt4,
       6.*dt,  12.*dt2, 20.*dt3;

  Eigen::MatrixXd v(3,1);
  const dvector & end(range.second);
  v << end[0] - (coeff[0] + coeff[1] * dt + coeff[2] * dt2),
       end[1] - (coeff[1] + 2.*coeff[2] * dt),
       end[2] - (2.*coeff[2]);

  MinJerk::solve(coeff, A, v);
}

void MinJerk::solve1(dvector &coeff, const TimeRange &time_span, const VecRange &range)
{
  Eigen::MatrixXd A(3,3);
  double dt(time_span.second-time_span.first);
  double dt2(dt*dt);
  double dt3(dt2*dt);
  double dt4(dt2*dt2);
  A << 3.*dt2,  4.*dt3,  5.*dt4,
       6.*dt,  12.*dt2, 20.*dt3,
       6.,     24.*dt,  60.*dt2;

  Eigen::MatrixXd v(3,1);
  const dvector & end(range.second);
  v << end[0] - (coeff[1] + 2.*coeff[2] * dt),
       end[1] - (2.*coeff[2]),
       end[2];

  MinJerk::solve(coeff, A, v);
}

double MinJerk::eval(const dvector &coeff, double time)
{
  double result(coeff[5]);
  for (int i(4); i>=0; --i) result = time*result + coeff[i];
  return result;
}

dvector MinJerk::eval_full(const dvector &coeff, double time)
{
  dvector result({coeff[5], 5.*coeff[5], 20.*coeff[5]});
  for (int i(4); i>=0; --i) result[0] = time*result[0] + coeff[i];
  for (int i(4); i>=1; --i) result[1] = time*result[1] + double(i)*coeff[i];
  for (int i(4); i>=2; --i) result[2] = time*result[2] + double(i)*double(i-1)*coeff[i];
  return result;
}

MinJerk::MinJerk(const TimeRange &time_span_, const VecRange &s_range_, const VecRange &d_range_, const HighwayMap &hwmap_, int type_)
  : hwmap(hwmap_), time_span(time_span_), s_range(s_range_), d_range(d_range_), type(type_), s_coeff(6), d_coeff(6)
{
  // initial conditions fix first three coefficients
  s_coeff[0] = HighwayMap::range_s(s_range_.first[0]);
  s_coeff[1] = s_range_.first[1];
  s_coeff[2] = s_range_.first[2] * 0.5;
  d_coeff[0] = d_range_.first[0];
  d_coeff[1] = d_range_.first[1];
  d_coeff[2] = d_range_.first[2] * 0.5;
  // solve linear system of 3 eqaution to fix remaining three coefficients
  MinJerk::MinJerk::solve0(d_coeff, time_span_, d_range_);
  // which type of polynomial?
  if (type_==0)
  {
    // take into account that s is a loop
    VecRange s_range__(s_range);
    s_range__.first[0]=HighwayMap::range_s(s_range_.first[0]);
    if (HighwayMap::is_ahead(s_range__.first[0], s_range__.second[0]))
      while (s_range__.second[0]<s_range__.first[0]) s_range__.second[0]+=HighwayMap::max_s;
    MinJerk::solve0(s_coeff, time_span_, s_range__);
  }
  else if (type_==1)
    // hint: the final conditions for s now are given by speed, acc and jerk!
    MinJerk::solve1(s_coeff, time_span_, s_range_);
  else
  {
    std::cerr<<"MinJerk::MinJerk: invalid type "<<type_<<"."<<std::endl;
    throw -1;
  }
}

double MinJerk::s(double time) const
{
  double s = MinJerk::eval(s_coeff, time-time_span.first);
  if (s>HighwayMap::max_s) s-=HighwayMap::max_s;
  if (s<0) s+=HighwayMap::max_s;
  return s;
}

double MinJerk::d(double time) const
{
  return MinJerk::eval(d_coeff, time-time_span.first);
}

dvector MinJerk::sd(double time) const
{
  return {s(time), d(time)};
}

dvector MinJerk::operator()(double time) const
{
  return hwmap.getSmoothXY(sd(time));
}

dvector MinJerk::sd_full(double time) const
{
  dvector sfull(MinJerk::eval_full(s_coeff, time-time_span.first));
  dvector dfull(MinJerk::eval_full(d_coeff, time-time_span.first));
  dvector result(6); for (int i(0); i<3; ++i) { result[i]=sfull[i]; result[i+3]=dfull[i]; }
  return result;
}

}
