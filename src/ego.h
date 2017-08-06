#ifndef EGO_H
#define EGO_H

#include "data.h"
#include "predictions.h"
#include "records.h"
#include "highwaymap.h"
#include "trajectory.h"
#include <set>

class Ego
{
private:
  class Point {
  public:
    double time;
    dvector xy;
    Trajectory::MinJerk * min_jerk;
    Point(double time_, const dvector & xy_, Trajectory::MinJerk * t);
    ~Point();
  };

  double base_time;
  // goal speed (49.5 Mph)
  const double goal_speed  = 49.5*1.609344/3.6;  // m/s

  const HighwayMap & hwmap;

  std::map<double, Trajectory::MinJerk *> trajectories; // key: left end of the time range
  std::vector<Point> storage;

  void generate_initial_trajectory(const Telemetry &t);
  Trajectory::MinJerk *find_trajectory(double time) const;
  Trajectory::MinJerk *generate_successor_trajectory(Trajectory::MinJerk * trajectory, double time);
  // keep all trajectories on the list that are still in use and remove all the others
  void keep_trajectories(const std::set<Trajectory::MinJerk*> & keep);
public:
  Ego(const HighwayMap & hwmap_);
  ~Ego();
  void re_init();
  Response path(const Telemetry &t, const Records & records, const Predictions::Predictions & predictions);
};

#endif // EGO_H
