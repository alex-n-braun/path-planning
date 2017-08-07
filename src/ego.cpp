#include "ego.h"

const double Ego::goal_speed  = 49.25*1.609344/3.6;  // m/s
const double Ego::min_dist(6.*4.);  // 6 cars
const double Ego::max_dist(3.*min_dist);

bool Ego::check_keep_conditions(const Ego::Point &p, const Predictions::Predictions &predictions) const
{
  bool keep(true);

  /* HINT:
   * at the moment, the keep condition check only works if the heading car is slower.
   * what if the heading car accelerates again? the point within max_dist are
   * still safe, put perhaps they could be faster; think about a remove condition
   * for these
   */

  /* find the car in front of us and in our lane, that is too
   * close. Is there a record of such a car? Then the keep-conditions
   * are not fulfilled.
   */
  double start_s(p.sd[0]);
  double start_d(p.sd[1]);
  double des_d(desired_d);
  std::pair<double, double> d_rl_boundaries(start_d-3., des_d+3.); // boundaries of lane dest_lane, including some safety distance
  for (auto predn_(predictions.preds.cbegin()); keep & predn_!=predictions.preds.cend(); ++predn_)
  {
    Predictions::Prediction * predn  = predn_->second;
    dvector pos(predn->trajectory(p.time-base_time));
    dvector sd(hwmap.getSmoothFrenet(pos));
    if (sd[1]>d_rl_boundaries.first & sd[1]<d_rl_boundaries.second)
      // yes, the car is on my lane!
    {
      double delta_s(HighwayMap::distance_s(start_s, sd[0]));
      keep = !(delta_s>0. & delta_s<Ego::max_dist);
    }
  }

  return keep;
}

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
  // for initial state, assume lane following with maximum speed
  int dest_lane(t.car_d/4.);
  // destination (desired) d
  double des_d(2.+4.+3.6*double(dest_lane-1));
  desired_d = des_d;

  // speed in s direction
  // double goal_rescale(1.0/(1.0+curvature*des_d));
  double goal_speed_s(desired_speed);


  // jerk-minimizing trajectory with unspecified final s
  std::cout<<"initial trajectory: start-s: "<<start_s<<std::endl;
  Trajectory::TimeRange time_range(0., 5.5);
  Trajectory::MinJerk * min_jerk = new Trajectory::MinJerk(time_range,
                               Trajectory::VecRange({start_s, start_speed_s, start_accel_s},
                                        {goal_speed_s, 0., 0.}),
                               Trajectory::VecRange({start_d, start_speed_d, start_accel_d},
                                        {des_d, 0., 0.}), hwmap, 1, dt);
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
  }
  else
    return NULL;
}

Ego::Plan * Ego::generate_plan(const Ego::Point & point, const Predictions::Predictions & predictions, double delta_t)
{

//  return NULL;

  // if the last point is ahead of the goal_time, quit
  if (point.time+delta_t>=base_time+time_horizon)
    return NULL;

  // generate future ego for lane following
  {
    // find the car in front of us and in our lane. Is there a record of such a car?
    double delta_s_closest(1.0/0.0); // distance to the next car
    // HINT: remove next line?
    int id_closest(-1); // id of the closest car
    double start_s(point.sd[0]);
    double start_d(point.sd[1]);
    double des_d(desired_d);
    dvector pos;
    Predictions::Prediction * pred(NULL);
    std::pair<double, double> d_rl_boundaries(start_d-3., des_d+3.); // boundaries of lane dest_lane, including some safety distance
    for (auto p(predictions.preds.cbegin()); p!=predictions.preds.cend(); ++p)
    {
      Predictions::Prediction * predn_  = p->second;
      dvector pos_(predn_->trajectory(point.time-base_time));
      dvector sd(hwmap.getSmoothFrenet(pos_));
      if (sd[1]>d_rl_boundaries.first & sd[1]<d_rl_boundaries.second)
        // yes, the car is on my lane! but is it the one that is next to me?
      {
        double delta_s(HighwayMap::distance_s(start_s, sd[0]));

        if (delta_s>0 & delta_s<delta_s_closest)
        {
          delta_s_closest=delta_s;
          id_closest=p->first;
          pred=predn_;
          pos=pos_;
        }
      }
    }

    double scale(1.);
    double des_speed_(goal_speed);
    if (id_closest>=0)
    {
      // predict position
      double pred_s(hwmap.getSmoothFrenet(pos)[0]);
      // distance
      double dist_s(hwmap.distance_s(start_s,pred_s));
      /* specification for acceleration planning: at a distance min_dist,
       * the ego speed should equal the speed of the car ahead of the ego.
       * at a distance max_dist or more, ego may drive at its goal_speed.
       * In both situations, acceleration should be zero.
       * In between, the ego's speed should be interpolated "smoothly".
       */
      scale=((dist_s-min_dist)/(max_dist-min_dist));
      scale=(scale>1. ? 1. : scale); // 0<=scale<=1
      if (scale>=0)
        scale=1.-0.5*(1.+cos(M_PI*scale)); // still 0<=scale<=1, but S-shaped
      else
        scale=-scale*scale; // don't want dist_s to be smaller then min_dist!
      des_speed_ = scale*goal_speed + (1.-scale)*lenvec(pred->trajectory.tangent(point.time-base_time));
      des_speed_ = (des_speed_<0. ? 0. : des_speed_);
    }

    Ego * future1 = new Ego(*this, base_time, time_horizon, des_speed_, desired_d, delta_t);
    Trajectory::MinJerk * new_trajectory(future1->generate_successor_trajectory(StateMachine(), point.trajectory, point.time));
    dvector sd(new_trajectory->sd(point.time+delta_t));
    dvector xy(hwmap.getSmoothXY(sd));

    Ego::Point newpoint(point.time+delta_t, StateMachine(), xy, sd, new_trajectory);

    double new_delta_t = dmin(delta_t*1.1, (new_trajectory->time_span.second-new_trajectory->time_span.first)*(1.-1.e-12));
    Ego::Plan * subplan = future1->generate_plan(newpoint, predictions, new_delta_t);

    return new Ego::Plan(subplan, future1);
  }
}

Trajectory::MinJerk *Ego::generate_successor_trajectory(const StateMachine & state, Trajectory::MinJerk *trajectory, double time)
{
  // erase all trajectories that are in the future
  double eps(dt*0.1);
  for (auto future(trajectories.lower_bound(time-eps)); future!=trajectories.end(); future++)
    delete future->second;
  trajectories.erase(trajectories.lower_bound(time-eps), trajectories.end());


  // check state

  switch (state.state)
  {
  case StateMachine::lanefollow:
  {
    dvector sdfull(trajectory->sd_full(time));
    double start_s, start_speed_s, start_accel_s,
           start_d, start_speed_d, start_accel_d;

    start_s = sdfull[0]; start_speed_s = sdfull[1]; start_accel_s = sdfull[2];
    std::cout<<"generate_successor_trajectory(): successor start_s: "<<start_s<<""<<std::endl;
    start_d = sdfull[3]; start_speed_d = sdfull[4]; start_accel_d = sdfull[5];
    Trajectory::TimeRange time_range(time+0., time+lane_follow_time);
    Trajectory::MinJerk * new_trajectory = new Trajectory::MinJerk(time_range,
                                 Trajectory::VecRange({start_s, start_speed_s, start_accel_s},
                                          {desired_speed, 0., 0.}),
                                 Trajectory::VecRange({start_d, start_speed_d, start_accel_d},
                                          {desired_d, 0., 0.}), hwmap, 1, dt);
    trajectories[time_range.first] = new_trajectory;

    return new_trajectory;
  }
  }
}

void Ego::keep_trajectories(const std::set<Trajectory::MinJerk *> &keep)
{
  for (auto trajectory(trajectories.begin()); trajectory!=trajectories.end(); ++trajectory)
  {
    if (keep.count(trajectory->second)==0)
    {
      delete trajectory->second;
      trajectories.erase(trajectory);
    }
  }
}

Ego::Ego(const HighwayMap &hwmap_, double desired_speed_, double dt_, double time_horizon_)
  : parent(NULL), hwmap(hwmap_), base_time(0.), time_horizon(time_horizon_), desired_speed(desired_speed_),
    dt(dt_)
{
  re_init();
}

Ego::Ego(const Ego &parent_, double base_time_, double time_horizon_, double desired_speed_, double desired_d_, double dt_)
  : parent(&parent_), hwmap(parent_.hwmap), base_time(base_time_), time_horizon(time_horizon_),
    desired_speed(desired_speed_), desired_d(desired_d_), dt(dt_)
{

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

Response Ego::path(const Telemetry &t, const Predictions::Predictions &predictions)
{
  Response r;

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*.
   */

  int path_size = t.previous_path_x.size();
  int max_path_size(time_horizon / dt); //(init==0 | path_size==0 ? 200 : path_size);

  const int consumed_steps(max_path_size-path_size);
  const int max_keep_steps(dmin(25, path_size-1));
  const int min_keep_steps(consumed_steps>3 ? 3*consumed_steps : 10);
  if (path_size==0)
    base_time = 0;
  else
    base_time += double(consumed_steps)*dt;

  std::set<Trajectory::MinJerk*> used_trajectories;

  //  path_size = (path_size > keep_steps ? keep_steps : path_size);
  int keep_steps(0);

  StateMachine state;
  if (storage.size()>0)
  {
    storage.erase(storage.begin(), storage.begin()+consumed_steps);
//    auto s(storage.begin());
//    std::cout<<"1st Storage element: ("<<s->xy[0]<<", "<<s->xy[1]<<") == "
//             <<"1st old element: ("<<t.previous_path_x[0]<<", "<<t.previous_path_y[0]<<") ???"
//             <<std::endl;
//    s=storage.end(); s--;
//    std::cout<<"last Storage element: ("<<s->xy[0]<<", "<<s->xy[1]<<") == "
//             <<"last old element: ("<<t.previous_path_x[path_size-1]<<", "<<t.previous_path_y[path_size-1]<<") ???"
//             <<std::endl;

    double delta_keep_steps(1.1);
    /* dont check every point; that take too much time;
     * instead, increase the delta between points every time
     * a check was performed
     */
    auto p(storage.begin());
    for (; (keep_steps<min_keep_steps
            ? true
            : (
               ((p!=storage.end())
                 ? (keep_steps % int(delta_keep_steps) == 0 ? delta_keep_steps*=1.2, check_keep_conditions(*p, predictions) : true)
                 : false)
                & keep_steps<max_keep_steps)); ++p)
      /* HINT: there is a problem with the storage when the
       * complete history is used; therefore at least one
       * element at the end of the storage is removed
       * (keep_steps<path_size-1). With that work-around, it
       * works. Still need to figure out what is exactly the
       * problem.
       */
    {
      r.next_x_vals.push_back(p->xy[0]);
      r.next_y_vals.push_back(p->xy[1]);
      used_trajectories.insert(p->trajectory);
      keep_steps++;
    }

    if (p!=storage.end())
    {
      storage.erase(p, storage.end());
    }
    state = storage.back().state;

    if (keep_steps<path_size-1)
    {
      std::cout<<"kept storage points: "<<keep_steps<<" < "<<path_size<<" (consumed steps: "<<consumed_steps<<")"<<std::endl;
      Plan * plan(generate_plan(storage.back(), predictions, 0.5));


      if (plan!=NULL)
      {
        std::cout<<"*** We have a plan! ****"<<std::endl;
        // erase all trajectories that are in the future
        double eps(dt*0.1);
        double time_(storage.back().time-eps);
        for (auto future(trajectories.lower_bound(time_)); future!=trajectories.end(); future++)
          delete future->second;
        trajectories.erase(trajectories.lower_bound(time_), trajectories.end());
      }

      Plan * iterate(plan);
      while (iterate!=NULL)
      {
        Trajectory::MinJerk * next_trajectory(new Trajectory::MinJerk(*(iterate->ego->trajectories.begin()->second)));
//        Trajectory::MinJerk * next_trajectory(iterate->ego->trajectories.begin()->second);
        next_trajectory->resample(dt);
        trajectories[next_trajectory->time_span.first] = next_trajectory;

        Plan * sub_plan(iterate->sub_plan);
        iterate=sub_plan;
      }

      delete plan;
    }
  }
  else
    state = StateMachine();

//  std::cout<<"consumed_steps: "<<consumed_steps<<", "<<path_size<<", "
//           <<"base time: "<<base_time<<"s, "
//           <<"storage size: "<<storage.size()<<std::endl;

  // initial trajectory
  if (trajectories.empty()) generate_initial_trajectory(t);

  {
    Trajectory::MinJerk * trajectory(NULL);

    for (int i(1); i<=max_path_size-keep_steps; ++i)
    {
      double time(base_time+double(keep_steps+i)*dt);
      Trajectory::MinJerk * next_trajectory = find_trajectory(time);
      if (next_trajectory==NULL)
      /* there is no trajectory containing the time in its range
       * or a state transition has occured
       */
      {
        next_trajectory = generate_successor_trajectory(state, trajectory, time-dt);
      }

      trajectory=next_trajectory;
      used_trajectories.insert(trajectory);

      dvector sd(trajectory->sd(time));
      dvector xy(hwmap.getSmoothXY(sd));
      r.next_x_vals.push_back(xy[0]);
      r.next_y_vals.push_back(xy[1]);

      storage.push_back(Point(time, state, xy, sd, trajectory));
    }

    // keep all trajectories on the list that are still in use and remove all the others
    keep_trajectories(used_trajectories);
  }

  return r;
}


Ego::Point::Point(double time_, const StateMachine &state_, const dvector &xy_, const dvector &sd_, Trajectory::MinJerk *t)
  : time(time_), xy(xy_), sd(sd_), trajectory(t), state(state_)
{

}

Ego::Point::~Point()
{
}

Ego::Plan::Plan(Ego::Plan *sub_plan_, Ego * ego_)
  : sub_plan(sub_plan_), ego(ego_)
{

}

Ego::Plan::~Plan()
{
  delete sub_plan;
  delete ego;
}

