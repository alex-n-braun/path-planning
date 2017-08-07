#ifndef EGO_H
#define EGO_H

#include "data.h"
#include "predictions.h"
#include "highwaymap.h"
#include "trajectory.h"
#include "statemachine.h"
#include <set>

class Ego
{
public:
  // goal speed (49.5 Mph)
  static const double goal_speed;  // m/s
  static const double min_dist;    // m
  static const double max_dist;    // m
private:
  class Point {
  public:
    double time;
    dvector xy;
    dvector sd;
    StateMachine state;
    Trajectory::MinJerk * trajectory;
    Point(double time_, const StateMachine & state_, const dvector & xy_, const dvector & sd_, Trajectory::MinJerk * t);
    ~Point();
  };
  class Plan{
  public:
    double score;
    Plan * sub_plan;
    Ego * ego;
    Plan(Plan * sub_plan_, Ego * ego_);
    ~Plan();
  };

  const Ego * parent;

  double base_time;
  double time_horizon;
  const double lane_follow_time = 4.0;
  double desired_speed;
  double desired_d;
  double dt;

  const HighwayMap & hwmap;

  std::map<double, Trajectory::MinJerk *> trajectories; // key: left end of the time range
  std::vector<Point> storage;


  // for a point p in the storage, check if it can be kept according to the predictions
  bool check_keep_conditions(const Point & p, const Predictions::Predictions &predictions) const;

  void generate_initial_trajectory(const Telemetry &t);
  Trajectory::MinJerk *generate_successor_trajectory(const StateMachine & state, Trajectory::MinJerk * trajectory, double time);
  // keep all trajectories on the list that are still in use and remove all the others
  void keep_trajectories(const std::set<Trajectory::MinJerk*> & keep);
protected:
  Trajectory::MinJerk *find_trajectory(double time) const;
  Plan * generate_plan(const Point &point, const Predictions::Predictions &predictions, double delta_t);  // returns a plan
public:
  Ego(const HighwayMap & hwmap_, double desired_speed_ = Ego::goal_speed, double dt_ = 0.02, double time_horizon_ = 7.4);
  Ego(const Ego & parent_, double base_time_, double time_horizon_, double desired_speed_, double desired_d_, double dt_);
  ~Ego();
  void re_init();
  Response path(const Telemetry &t, const Predictions::Predictions & predictions);
};

#endif // EGO_H
