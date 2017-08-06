#include "ego.h"

void Ego::generate_initial_trajectory(const Telemetry &t)
{
  // start state
  double start_s, start_speed_s, start_accel_s,
         start_d, start_speed_d, start_accel_d;
  dvector start_sd({t.car_s, t.car_d});
  start_s = start_sd[0];
  start_d = start_sd[1];
  dvector v_start_speed({t.car_speed*cos(t.car_yaw), t.car_speed*sin(t.car_yaw)});
  // speed in tangential direction of lane
  double start_speed_t = projectionlength(hwmap.tangent(start_s), v_start_speed);
  double curvature(hwmap.map_trajectory.curvature(start_s));
  double start_rescale(1.0/(1.0+curvature*start_d));
  start_speed_s = start_speed_t * start_rescale;
  // orthogonal component
  start_speed_d = projectionlength(hwmap.orthogonal(start_s), v_start_speed);
  // accelerations
  start_accel_s=0.; start_accel_d=0.;

  // destination lane (0: left lane, 1: middle, 2: right lane)
  int dest_lane(0);
  // destination (desired) d
  double des_d(2.+4.+3.6*double(dest_lane-1));

  // speed in s direction
  // double goal_rescale(1.0/(1.0+curvature*des_d));
  double goal_speed_s(goal_speed);


  // jerk-minimizing trajectory with unspecified final s
  std::cout<<"initial trajectory: start-s: "<<start_s<<std::endl;
  Trajectory::TimeRange time_range(0., 5.5);
  Trajectory::MinJerk * min_jerk = new Trajectory::MinJerk(time_range,
                               Trajectory::VecRange({start_s, start_speed_s, start_accel_s},
                                        {goal_speed_s, 0., 0.}),
                               Trajectory::VecRange({start_d, start_speed_d, start_accel_d},
                                        {des_d, 0., 0.}), hwmap, 1);
  trajectories[time_range.first] = min_jerk;
}

Trajectory::MinJerk *Ego::find_trajectory(double time) const
{
  if (trajectories.size()>0)
  {
    auto t(trajectories.lower_bound(time));
    t--;
    if (t->second->is_time_in_range(time))
      return t->second;
    else
      return NULL;

//    if (min_jerk->is_time_in_range(time))
//      return min_jerk;
//    else
//      return NULL;
  }
  else
    return NULL;
}

Trajectory::MinJerk *Ego::generate_successor_trajectory(Trajectory::MinJerk *trajectory, double time)
{
  dvector sdfull(trajectory->sd_full(time));
  double start_s, start_speed_s, start_accel_s,
         start_d, start_speed_d, start_accel_d;

  // destination lane (0: left lane, 1: middle, 2: right lane)
  int dest_lane(2);
  // destination (desired) d
  double goal_d(2.+4.+3.6*double(dest_lane-1));
  start_s = sdfull[0]; start_speed_s = sdfull[1]; start_accel_s = sdfull[2];
  std::cout<<"generate_successor_trajectory(): successor start_s: "<<start_s<<""<<std::endl;
  start_d = sdfull[3]; start_speed_d = sdfull[4]; start_accel_d = sdfull[5];
  Trajectory::TimeRange time_range(time+0., time+2.5);
  Trajectory::MinJerk * new_trajectory = new Trajectory::MinJerk(time_range,
                               Trajectory::VecRange({start_s, start_speed_s, start_accel_s},
                                        {goal_speed, 0., 0.}),
                               Trajectory::VecRange({start_d, start_speed_d, start_accel_d},
                                        {goal_d, 0., 0.}), hwmap, 1);
  trajectories[time_range.first] = new_trajectory;
  return new_trajectory;
}

void Ego::keep_trajectories(const std::set<Trajectory::MinJerk *> &keep)
{
  for (auto trajectory(trajectories.begin()); trajectory!=trajectories.end(); ++trajectory)
  {
    if (keep.count(trajectory->second)==0)
    {
      std::cout<<"delete trajectory ("<<trajectory->first<<", "<<trajectory->second->time_span.second<<")"<<std::endl;
      delete trajectory->second;
      trajectories.erase(trajectory);
    }
  }
}

Ego::Ego(const HighwayMap &hwmap_)
  : hwmap(hwmap_), base_time(0.)
{
  re_init();
}

Ego::~Ego()
{
  for (auto t(trajectories.cbegin()); t!=trajectories.cend(); ++t)
    delete t->second;
}

void Ego::re_init()
{
  base_time=0.;
  for (auto t(trajectories.cbegin()); t!=trajectories.cend(); ++t)
    delete t->second;
  trajectories.clear();
  storage.clear();
}

Response Ego::path(const Telemetry &t, const Records &records, const Predictions::Predictions &predictions)
{
  Response r;

  const double dt(0.02);

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*.
   */

  int path_size = t.previous_path_x.size();
  int keep_steps(25); // (init==0 ? 50 : 0);
  int max_path_size(70); //(init==0 | path_size==0 ? 200 : path_size);

  int consumed_steps(max_path_size-path_size);
  if (path_size==0)
    base_time = 0;
  else
    base_time += double(consumed_steps)*dt;
//  std::cout<<"consumed_steps: "<<consumed_steps<<", "<<path_size<<", "
//           <<"base time: "<<base_time<<"s, "
//           <<"storage size: "<<storage.size()<<std::endl;

  path_size = (path_size > keep_steps ? keep_steps : path_size);
  keep_steps = path_size;

  if (storage.size()>0)
  {
    storage.erase(storage.begin(), storage.begin()+consumed_steps);
    storage.erase(storage.begin()+path_size, storage.end());
    auto s(storage.begin());
//    std::cout<<"1st Storage element: ("<<s->xy[0]<<", "<<s->xy[1]<<") == "
//             <<"1st old element: ("<<t.previous_path_x[0]<<", "<<t.previous_path_y[0]<<") ???"
//             <<std::endl;
//    s=storage.end(); s--;
//    std::cout<<"last Storage element: ("<<s->xy[0]<<", "<<s->xy[1]<<") == "
//             <<"last old element: ("<<t.previous_path_x[path_size-1]<<", "<<t.previous_path_y[path_size-1]<<") ???"
//             <<std::endl;
    for (auto p(storage.cbegin()); p!=storage.cend(); ++p)
    {
      r.next_x_vals.push_back(p->xy[0]);
      r.next_y_vals.push_back(p->xy[1]);
    }
  }

  // initial trajectory
  if (trajectories.empty()) generate_initial_trajectory(t);

  {
    Trajectory::MinJerk * trajectory;

    std::set<Trajectory::MinJerk*> used_trajectories;

    for (int i(1); i<=max_path_size-path_size; ++i)
    {
      double time(base_time+double(path_size)*dt+double(i)*dt);
      Trajectory::MinJerk * next_trajectory = find_trajectory(time);
      if (next_trajectory==NULL) // there is no trajectory containing the time in its range
      {
        next_trajectory = generate_successor_trajectory(trajectory, time-dt);
      }
      trajectory=next_trajectory;
      used_trajectories.insert(trajectory);
      dvector xy((*trajectory)(time));
      r.next_x_vals.push_back(xy[0]);
      r.next_y_vals.push_back(xy[1]);

      storage.push_back(Point(time, xy, trajectory));
    }

    // keep all trajectories on the list that are still in use and remove all the others
    keep_trajectories(used_trajectories);
  }

  return r;
}


Ego::Point::Point(double time_, const dvector &xy_, Trajectory::MinJerk *t)
  : time(time_), xy(xy_), min_jerk(t)
{

}

Ego::Point::~Point()
{
}
