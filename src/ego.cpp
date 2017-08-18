#include "ego.h"

const double Ego::goal_speed  = 49.25*1.609344/3.6;  // m/s
const double Ego::min_dist(6.*4.);  // 6 cars
const double Ego::max_dist(3.*min_dist);

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

double Ego::calc_safe_speed(double start_s, double car_ahead_s, double car_ahead_speed) const
{
  double scale(1.);
  // distance
  double dist_s(hwmap.distance_s(start_s,car_ahead_s));
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
  double des_speed_(scale*goal_speed + (1.-scale)*car_ahead_speed);
  des_speed_ = (des_speed_<0. ? 0. : des_speed_);

  return des_speed_;
}

Ego::Car_Ahead Ego::find_car_ahead(double point_time, double start_s, double start_d, double des_d, const Predictions::Predictions & predictions) const
{
  // find the car in front of us in our desired lane. Is there a record of such a car?
  double delta_s_closest(1.0/0.0); // distance to the next car

  int id_closest(-1); // id of the closest car

  double left_d(start_d);
  double right_d(des_d);
  dsort(left_d, right_d); // "left" is really left of right

  dvector pos;
  Predictions::Prediction * pred(NULL);

  std::pair<double, double> d_rl_boundaries(left_d-2., right_d+2.); // boundaries of the lane
  for (auto p(predictions.preds.cbegin()); p!=predictions.preds.cend(); ++p)
  {
    Predictions::Prediction * predn_  = p->second;
    dvector pos_(predn_->trajectory(point_time-base_time));
    dvector sd(hwmap.getSmoothFrenet(pos_));
    if (sd[1]>d_rl_boundaries.first & sd[1]<d_rl_boundaries.second)
      // yes, the car is on my lane! but is it the one that is next to me?
    {
      double delta_s(HighwayMap::distance_s(start_s, sd[0]));

      if (delta_s>0. & delta_s<delta_s_closest)
      {
        delta_s_closest=delta_s;
        id_closest=p->first;
        pred=predn_;
        pos=pos_;
      }
    }
  }

  return Car_Ahead(Predictions::Predictions::ID_Prediction(id_closest, pred), pos);
}

dvector Ego::get_initial_conditions(const Point & point) const
{
  if (point.trajectory!=NULL)
    return point.trajectory->sd_full(point.time);
  else
  {
    return {point.sd[0], 0, 0, point.sd[1], 0, 0};
  }
}

Ego::Plan * Ego::lane_follow(const Ego::Point & point, const Predictions::Predictions & predictions, double delta_t)
{
  StateMachine new_state(StateMachine::lanefollow, point.state.dest_lane);

  double start_s(point.sd[0]);
  double start_d(point.sd[1]);
  double des_d(hwmap.get_d_from_lane(point.state.dest_lane));

  // find the car in front of us and in our lane. Is there a record of such a car?
  Car_Ahead car_ahead(find_car_ahead(point.time, start_s, start_d, des_d, predictions));
  int id_closest(car_ahead.first.first); // id of the closest car

  Predictions::Prediction * pred(car_ahead.first.second);
  dvector pos(car_ahead.second);

  /* now that I found the car in front of me (in case id_closes>=0),
   * I want to compute a speed for myself that is safe to go.
   */
  double des_speed_(id_closest>=0
                    ? calc_safe_speed(start_s, hwmap.getSmoothFrenet(pos)[0], lenvec(pred->trajectory.tangent(point.time-base_time)))
                    : goal_speed);

  Ego * future1 = new Ego(*this, base_time, time_horizon, des_speed_, delta_t);
  future1->recursion_depth = recursion_depth + 1;

  Trajectory::MinJerk * new_trajectory(future1->generate_successor_trajectory(new_state, get_initial_conditions(point), point.time));
  dvector sd(new_trajectory->sd(point.time+delta_t));
  dvector xy(hwmap.getSmoothXY(sd));

  Ego::Point newpoint(point.time+delta_t, new_state, xy, sd, new_trajectory);

  double new_delta_t = dmin(delta_t*1.1, (new_trajectory->time_span.second-new_trajectory->time_span.first)*(1.-1.e-12));
  Ego::Plan * subplan = future1->generate_plan(newpoint, predictions, new_delta_t);

  Plan * plan(new Ego::Plan(new_state, subplan, future1, base_time + time_horizon));
  plan->score -= dabs(start_d-des_d);

  return plan;
}

Ego::Plan * Ego::lane_change(const Ego::Point & point, double des_d, const Predictions::Predictions & predictions, double delta_t)
{
  double start_s(point.sd[0]);
  double start_d(point.sd[1]);

  // find the car in front of us and in our lane. Is there a record of such a car?
  Car_Ahead car_ahead(find_car_ahead(point.time, start_s, start_d, des_d, predictions));
  int id_closest(car_ahead.first.first); // id of the closest car

  Predictions::Prediction * pred(car_ahead.first.second);
  dvector pos(car_ahead.second);

  /* now that I found the car in front of me (in case id_closes>=0),
   * I want to compute a speed for myself that is safe to go.
   */
  double des_speed_(id_closest>=0
                    ? calc_safe_speed(start_s, hwmap.getSmoothFrenet(pos)[0], lenvec(pred->trajectory.tangent(point.time-base_time)))
                    : goal_speed);

  StateMachine new_state(dabs(start_d-des_d)<0.5
                         ? StateMachine(StateMachine::lanefollow, hwmap.getLane(des_d))
                         : StateMachine(StateMachine::lanechange, hwmap.getLane(des_d)));

  Ego * future1 = new Ego(*this, base_time, time_horizon, des_speed_, delta_t);
  future1->recursion_depth = recursion_depth + 1;
  Trajectory::MinJerk * new_trajectory(future1->generate_successor_trajectory(new_state, get_initial_conditions(point), point.time));
  dvector sd(new_trajectory->sd(point.time+delta_t));
  dvector xy(hwmap.getSmoothXY(sd));

  Ego::Point newpoint(point.time+delta_t, new_state, xy, sd, new_trajectory);

  double new_delta_t = dmin(delta_t*1.1, (new_trajectory->time_span.second-new_trajectory->time_span.first)*(1.-1.e-12));
  Ego::Plan * subplan = future1->generate_plan(newpoint, predictions, new_delta_t);

  Plan * plan(new Ego::Plan(newpoint.state, subplan, future1, base_time + time_horizon));
  plan->score -= dabs(start_d-des_d);

  return plan;
}

// just for debugging
int max_recursion_depth = 0;

Ego::Plan * Ego::generate_plan(const Ego::Point & point, const Predictions::Predictions & predictions, double delta_t)
{
  if (recursion_depth > max_recursion_depth)
  {
    max_recursion_depth = recursion_depth;
    std::cout<<"Maximum recursion depth so far: "<<max_recursion_depth<<std::endl;
  }

  // if the last point is ahead of the goal_time, quit recursion
  if (point.time>=base_time+time_horizon)
    return NULL;

  switch (point.state.state)
  {
  case StateMachine::lanefollow:
  {
    std::vector<Plan * > plans(3);

    plans[0] = lane_follow(point, predictions, delta_t);

    // plans for lane changes
    for (int lc(1); lc<3; ++lc)
    // generate future ego for lane following
    {
      double des_d(hwmap.get_d_from_lane(point.state.dest_lane));
      int lane((hwmap.getLane(des_d)+lc) % 3);
      des_d = hwmap.get_d_from_lane(lane);

      plans[lc] = lane_change(point, des_d, predictions, delta_t);
    }

    // travelled distance rating
    double dist1(hwmap.distance_s(plans[0]->s, plans[1]->s));
    double dist2(hwmap.distance_s(plans[0]->s, plans[2]->s));
//    {
//      for (int i(0); i<3; ++i)
//        std::cout<<i<<": "<<plans[i]->s<<", ";
//      std::cout<<std::endl;
//    }
    int lc(0);
    if (dist1>dist2)
    {
      if (dist1>0)
        lc=1;
    }
    else if (dist2>0)
      lc=2;

    for (int i(0); i<3; ++i) if (i!=lc) delete plans[i];
    return plans[lc];
  }
  case StateMachine::lanechange:
  {
    return lane_change(point, hwmap.get_d_from_lane(point.state.dest_lane),
                       predictions, delta_t);
  }
  }
}

//  for (int lc(0); lc<1; ++lc)
//  // generate future ego for lane following
//  {
//    StateMachine new_state(StateMachine::lanefollow);
//    // find the car in front of us and in our lane. Is there a record of such a car?
//    double delta_s_closest(1.0/0.0); // distance to the next car
//    // HINT: remove next line?
//    int id_closest(-1); // id of the closest car
//    double start_s(point.sd[0]);
//    double start_d(point.sd[1]);
//    double des_d(desired_d);

//    double des_speed_(goal_speed);
//    double left_d(start_d); double right_d(des_d);

//    if (lc!=-1) // HINT: bullshit???? if lane is to be changed...
//    {
//      int lane((int(des_d/4)+lc) % 3);
//      des_d = 2. + 4. * double(lane);


//      dvector pos;
//      Predictions::Prediction * pred(NULL);

//      right_d = des_d;

//      dsort(left_d, right_d);
//      std::pair<double, double> d_rl_boundaries(left_d-3., right_d+3.); // boundaries of lane dest_lane, including some safety distance
//      for (auto p(predictions.preds.cbegin()); p!=predictions.preds.cend(); ++p)
//      {
//        Predictions::Prediction * predn_  = p->second;
//        dvector pos_(predn_->trajectory(point.time-base_time));
//        dvector sd(hwmap.getSmoothFrenet(pos_));
//        if (sd[1]>d_rl_boundaries.first & sd[1]<d_rl_boundaries.second)
//          // yes, the car is on my lane! but is it the one that is next to me?
//        {
//          double delta_s(HighwayMap::distance_s(start_s, sd[0]));

//          if (delta_s>(lc==0 ? 0. : -4.) & delta_s<delta_s_closest)
//          {
//            delta_s_closest=delta_s;
//            id_closest=p->first;
//            pred=predn_;
//            pos=pos_;
//          }
//        }
//      }

//      if (id_closest>=0)
//      {
//        des_speed_ = calc_safe_speed(start_s, hwmap.getSmoothFrenet(pos)[0], lenvec(pred->trajectory.tangent(point.time-base_time)));
//      }
//    }

//    Ego * future1 = new Ego(*this, base_time, time_horizon, des_speed_, des_d, delta_t);
//    Trajectory::MinJerk * new_trajectory(future1->generate_successor_trajectory(new_state, point.trajectory, point.time));
//    dvector sd(new_trajectory->sd(point.time+delta_t));
//    dvector xy(hwmap.getSmoothXY(sd));

//    Ego::Point newpoint(point.time+delta_t, new_state, xy, sd, new_trajectory);

//    double new_delta_t = dmin(delta_t*1.3, (new_trajectory->time_span.second-new_trajectory->time_span.first)*(1.-1.e-12));
//    Ego::Plan * subplan = future1->generate_plan(newpoint, predictions, new_delta_t);

//    plans[lc] = new Ego::Plan(subplan, future1);

//    plans[lc]->score -= dabs(right_d-left_d);
//  }

Trajectory::MinJerk *Ego::generate_successor_trajectory(const StateMachine & state, const dvector &ini_state, double time)
{
  // erase all trajectories that are in the future
  // HINT: since we delete ALL trajectories after we found our initial conditions,
  // this section is not needed anymore.
  double eps(dt*0.1);
  for (auto future(trajectories.lower_bound(time-eps)); future!=trajectories.end(); future++)
    delete future->second;
  trajectories.erase(trajectories.lower_bound(time-eps), trajectories.end());

  double start_s, start_speed_s, start_accel_s,
         start_d, start_speed_d, start_accel_d;

  start_s = ini_state[0]; start_speed_s = ini_state[1]; start_accel_s = ini_state[2];
  start_d = ini_state[3]; start_speed_d = ini_state[4]; start_accel_d = ini_state[5];
  Trajectory::TimeRange time_range(time+0., time+lane_follow_time*(1.+dabs(hwmap.get_d_from_lane(state.dest_lane)-start_d)/8.)*dmax(1., 1.+(desired_speed-start_speed_s)/20.));
  Trajectory::MinJerk * new_trajectory = new Trajectory::MinJerk(time_range,
                               Trajectory::VecRange({start_s, start_speed_s, start_accel_s},
                                        {desired_speed, 0., 0.}),
                               Trajectory::VecRange({start_d, start_speed_d, start_accel_d},
                                        {/*desired_d*/hwmap.get_d_from_lane(state.dest_lane), 0., 0.}), hwmap, 1, dt);
  trajectories[time_range.first] = new_trajectory;

  return new_trajectory;
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

Ego::Ego(const Ego &parent_, double base_time_, double time_horizon_, double desired_speed_, /*double desired_d_, */double dt_)
  : parent(&parent_), hwmap(parent_.hwmap), base_time(base_time_), time_horizon(time_horizon_),
    desired_speed(desired_speed_), /*desired_d(desired_d_), */dt(dt_)
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
  int max_path_size(time_horizon / dt -1); //(init==0 | path_size==0 ? 200 : path_size);

  const int consumed_steps(storage.size()-path_size);
  const int min_keep_steps(dmin(20, path_size));

  // initialize clock
  if (path_size==0)
    base_time = 0;
  else
    base_time += double(consumed_steps)*dt;

  std::set<Trajectory::MinJerk*> used_trajectories;

  //  path_size = (path_size > keep_steps ? keep_steps : path_size);
  int keep_steps(0);
  Point point;
  if (storage.size()>0)
  {
    // erase points from the storage that have been consumed by the simulator
    storage.erase(storage.begin(), storage.begin()+consumed_steps);

    /* erase all points behind min_keep_steps; they will be re-computed.
     * the other ones are reused. */
    auto p(storage.begin());
    for (; keep_steps<min_keep_steps; ++p)
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

    point = storage.back();
  }
  else // storage is empty; initial time step...
  {
    point = Point(base_time, StateMachine(StateMachine::lanefollow, hwmap.getLane(t.car_d)), {t.car_x, t.car_y}, {t.car_s, t.car_d}, NULL);
//    desired_d=t.car_d;
  }

//  if (point.state.state == StateMachine::lanechange)
//    std::cout<<"history says: lane change to "<<point.state.dest_lane<<std::endl;

//  plan_recursion_steps = 0;
  Plan * plan(generate_plan(point, predictions, 0.24));
//  std::cout<<plan_recursion_steps<<std::endl;
  // clear all trajectories from last computation step, since now we do not need them anymore
  keep_trajectories(used_trajectories);

  Plan * iterate(plan);
//  while (iterate!=NULL)
//  {
    Trajectory::MinJerk * next_trajectory(new Trajectory::MinJerk(*(iterate->ego->trajectories.begin()->second)));
    next_trajectory->resample(dt);
    trajectories[next_trajectory->time_span.first] = next_trajectory;

    Plan * sub_plan(iterate->sub_plan);
    iterate=sub_plan;
//  }

  StateMachine state(plan->state);
  if (state.state != point.state.state)
  {
    if (state.state == StateMachine::lanechange)
      std::cout<<"State Machine: lane change to "<<state.dest_lane<<std::endl;
    else
      std::cout<<"State Machine: keep lane"<<std::endl;
  }
  delete plan;


//  std::cout<<(state.state == StateMachine::lanefollow ? "lane follow" : "lane change")<<std::endl;

  for (int i(1); i<=max_path_size-keep_steps; ++i)
  {
    double time(base_time+double(keep_steps+i)*dt);

    Trajectory::MinJerk * trajectory = find_trajectory(time);
    if (trajectory==NULL)
    {
      // there is no trajectory containing the time in its range
      return r;
    }

    dvector sd(trajectory->sd(time));
    dvector xy(hwmap.getSmoothXY(sd));
    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);

    storage.push_back(Point(time, state, xy, sd, trajectory));
  }

  return r;
}


Ego::Point::Point(double time_, const StateMachine &state_, const dvector &xy_, const dvector &sd_, Trajectory::MinJerk *t)
  : time(time_), xy(xy_), sd(sd_), trajectory(t), state(state_) { }

Ego::Point::Point()
  : time(0), xy({0,0}), sd({0,0}), trajectory(NULL), state(StateMachine::lanefollow, 1) { }

Ego::Point::~Point() { }

Ego::Plan::Plan(const StateMachine &state_, Ego::Plan *sub_plan_, Ego * ego_, double max_s_time)
  : state(state_), sub_plan(sub_plan_), ego(ego_)
{
  if (sub_plan_==NULL)
  {
    Trajectory::MinJerk * t(ego->trajectories.begin()->second);
    s=t->s(max_s_time);
    score=0;
  }
  else
  {
    s = sub_plan->s;
    score = sub_plan->score;
  }
}

Ego::Plan::~Plan()
{
  delete sub_plan;
  delete ego;
}

