#ifndef EGO_H
#define EGO_H

#include "data.h"
#include "predictions.h"
#include "records.h"
#include "highwaymap.h"
#include "trajectory.h"

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

  const HighwayMap & hwmap;

  std::map<double, Trajectory::MinJerk *> trajectories; // key: left end of the time range
  std::vector<Point> storage;

  Trajectory::MinJerk *find_trajectory(double time) const;
  Trajectory::MinJerk *generate_successor_trajectory(Trajectory::MinJerk * trajectory, double time);
public:
  Ego(const HighwayMap & hwmap_);
  ~Ego();
  void re_init();
  Response path(const Telemetry &t, const Records & records, const Predictions::Predictions & predictions);
};

#endif // EGO_H
