#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <map>
#include "spline.h"
#include "helpers.h"
#include "Eigen-3.3/Eigen/Core"

class HighwayMap;

namespace Trajectory {

class Spline
{
private:
  tk::spline spline_x;
  tk::spline spline_y;
public:
  Spline();
  Spline(const dvector & time, const dvector & x, const dvector & y);
  Spline(const Spline & spline);
  dvector operator()(double time) const;
  dvector tangent(double time) const;
  dvector normal(double time) const;
  dvector oriented_curvature(double time) const;
  double curvature(double time) const;
};

typedef std::pair<double, double> TimeRange;
typedef std::pair<dvector, dvector> VecRange;

class MinJerk {
private:
  const HighwayMap & hwmap;
  VecRange s_range;
  VecRange d_range;
  int type;
  dvector s_coeff;
  dvector d_coeff;

  static void solve(dvector & coeff, const Eigen::MatrixXd & A, const Eigen::MatrixXd & v);
  static void solve0(dvector & coeff, const TimeRange & time_span, const VecRange & range);
  static void solve1(dvector & coeff, const TimeRange & time_span, const VecRange & range);
  static double eval(const dvector & coeff, double time);
  static dvector eval_full(const dvector & coeff, double time);
  // data storage
  std::vector<dvector> data;
  dvector sd_full_calc(double time) const;
  void generate();
  int index_time(double time) const;
public:
  double dt;
  TimeRange time_span;
  MinJerk();
  /* compute a trajectory polynomial for s and d with fixed boundary conditions.
   * hwmap is needed to allow for transformation to xy.
   * type: type of final conditions.
   *    0: s, speed, acceleration given
   *    1: speed, acceleration, jerk given (no obstacles on the road? then why specify
   *       a final s?)
   */
  MinJerk(const TimeRange & time_span_, const VecRange & s_range_, const VecRange & d_range_, const HighwayMap & hwmap_, int type_, double dt_ = 0.02);
  MinJerk(const MinJerk & t); // INCOMPLETE copy constructor!!! call to resample is mandatory!
  ~MinJerk();
  // resample data with delta time dt_
  void resample(double dt_);
  double s(double time) const;
  double d(double time) const;
  dvector sd(double time) const;
  dvector operator()(double time) const;
  /* compute up to 2nd derivative of s and d and store in the following way: (s, s', s'', d, d', d'')
   */
  dvector sd_full(double time) const;
  /* compute time at which we are at s
   */
  double time(double s) const;
  /* is time in range?
   */
  bool is_time_in_range(double time) const;
};

}

#endif // TRAJECTORY_H
