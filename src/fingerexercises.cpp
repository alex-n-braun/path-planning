#include "fingerexercises.h"
#include "models.h"

Response fe_constspeed(const Telemetry &t)
{
  Response r;

  // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
  double dist_inc = 49.5*1.609344/3.6*0.02; //0.5;
  for(int i = 0; i < 50; i++)
  {
        r.next_x_vals.push_back(t.car_x+(dist_inc*double(i))*cos(deg2rad(t.car_yaw)));
        r.next_y_vals.push_back(t.car_y+(dist_inc*double(i))*sin(deg2rad(t.car_yaw)));
  }

  return r;
}

Response fe_circle(const Telemetry &t)
{
  Response r;

  double pos_x;
  double pos_y;
  double angle;

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*. */

  int path_size = t.previous_path_x.size();
  for(int i = 0; i < path_size; i++)
  {
      r.next_x_vals.push_back(t.previous_path_x[i]);
      r.next_y_vals.push_back(t.previous_path_y[i]);
  }

  /* if there was no previous path, take the current car's
   * position for initialisation. */
  if(path_size == 0)
  {
      pos_x = t.car_x;
      pos_y = t.car_y;
      angle = deg2rad(t.car_yaw);
  }
  else
  /* otherwise, take the last two positions from the old
   * path to compute the angle of the car... */
  {
      pos_x = t.previous_path_x[path_size-1];
      pos_y = t.previous_path_y[path_size-1];

      double pos_x2 = t.previous_path_x[path_size-2];
      double pos_y2 = t.previous_path_y[path_size-2];
      angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  /* ... and starting from the end of the old path, make
   * the planned path complete with 50 waypoints in total.
   * The car's yaw angle is increased by pi/100 for every
   * waypoint. This makes the car move on a circle. */
  double dist_inc = 0.5;
  for(int i = 0; i < 50-path_size; i++)
  {
    pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
    pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    r.next_x_vals.push_back(pos_x);
    r.next_y_vals.push_back(pos_y);
  }

  return r;
}

Response fe_waypoints(const Telemetry &t, const HighwayMap &m)
{
  Response r;

  int index(m.ClosestWaypoint(t.car_x, t.car_y));
  std::cout<<index<<", "<<t.car_x<<", "<<t.car_y<<", "<<t.car_s<<std::endl;

  for (int i(index); i<=index+50; ++i)
  {
    Waypoint w(m[i]);
    r.next_x_vals.push_back(w.x);
    r.next_y_vals.push_back(w.y);
  }

  return r;

}

Response fe_rightmostlane(const Telemetry &t, const HighwayMap &m)
{
  Response r;

  int index(m.ClosestWaypoint(t.car_x, t.car_y));
  std::cout<<index<<", "<<t.car_x<<", "<<t.car_y<<", "<<t.car_s<<std::endl;

  for (int i(0); i<=50; ++i)
  {
    double s(t.car_s+double(i)*(49.5*1.609344/3.6*0.02));
    double d(2.+4.+4.);

    dvector xy(m.getXY(s, d));

    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);
  }

  return r;

}

Response fe_smooth_rightmostlane(const Telemetry &t, const HighwayMap &m)
{
  Response r;

  int index(m.ClosestWaypoint(t.car_x, t.car_y));
  std::cout<<index<<", "<<t.car_x<<", "<<t.car_y<<", "<<t.car_s<<", "<<t.car_speed<<std::endl;

  for (int i(0); i<=150; ++i)
  {
    double s(t.car_s+double(i)*(49.0*1.609344/3.6*0.02));
    double d(2.+4.+4.);

    dvector xy(m.getSmoothXY(s, d));

    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);
  }

  return r;
}


Response fe_even_more_smooth_rightmostlane(const Telemetry &t, const HighwayMap &m)
{
  Response r;

  int index(m.ClosestWaypoint(t.car_x, t.car_y));
  std::cout<<index<<", "<<t.car_x<<", "<<t.car_y<<", "<<t.car_s<<", "<<t.car_speed<<std::endl;

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*. */

  int path_size = t.previous_path_x.size();
  for(int i = 0; i < path_size; i++)
  {
      r.next_x_vals.push_back(t.previous_path_x[i]);
      r.next_y_vals.push_back(t.previous_path_y[i]);
  }

  double start_s(path_size>0 ? m.getSmoothFrenet(t.previous_path_x[path_size-1], t.previous_path_y[path_size-1])[0]
      : t.car_s);

  for (int i(1); i<=150-path_size; ++i)
  {
    double s(start_s+double(i)*(49.0*1.609344/3.6*0.02));
    double d(2.+4.+4.);

    dvector xy(m.getSmoothXY(s, d));

    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);
  }

  return r;
}

Response fe_rightmostlane_constspeed(const Telemetry &t, const HighwayMap &m)
{
  Response r;

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*. */

  int path_size = t.previous_path_x.size();
  for(int i = 0; i < path_size; i++)
  {
      r.next_x_vals.push_back(t.previous_path_x[i]);
      r.next_y_vals.push_back(t.previous_path_y[i]);
  }

  double start_s(path_size>0 ? m.getSmoothFrenet(t.previous_path_x[path_size-1], t.previous_path_y[path_size-1])[0]
      : t.car_s);

  for (int i(1); i<=150-path_size; ++i)
  {
    double curvature(m.map_trajectory.curvature(start_s));
    double d(2.+4.+4.);
    double rescale(1.0/(1.0+curvature*d));

    double s(start_s+(49.0*1.609344/3.6*0.02)*rescale);

    dvector xy(m.getSmoothXY(s, d));

    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);

    start_s=s;

//    if (i==1)
//      std::cout<<index<<", "<<t.car_x<<", "<<t.car_y<<", "<<t.car_s<<", "<<t.car_speed<<", curve: "<<curvature<<", rescale: "<<rescale<<std::endl;
  }
//  for (auto s(t.sensor_fusion.begin()); s!=t.sensor_fusion.end(); ++s)
//    std::cout<<s->second<<std::endl;

  return r;
}

Response fe_rightmostlane_constdist(const Telemetry &t, const HighwayMap &m,
                                    const Records & records, const Predictions::Predictions & predictions)
{
  Response r;

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*. Here, we use
   * 25 steps from the old path. This corresponds to
   * 25*0.02s = 0.5s
   */

  int path_size = t.previous_path_x.size();
  path_size = (path_size > 25 ? 25 : path_size);
  for(int i = 0; i < path_size; i++)
  {
      r.next_x_vals.push_back(t.previous_path_x[i]);
      r.next_y_vals.push_back(t.previous_path_y[i]);
  }

  double start_s(path_size>0 ? m.getSmoothFrenet(t.previous_path_x[path_size-1], t.previous_path_y[path_size-1])[0]
      : t.car_s);

  double goal_speed(49.5*1.609344/3.6);  // m/s

  // find the car in front of us in the right-most lane. Is there a record of such a car?
  double delta_s_closest(1.0/0.0); // distance to the next car
  int id_closest(-1); // id of the closest car

  for (auto r(records.records.cbegin()); r!=records.records.cend(); ++r)
  {
    Detection detn = r->second.get_last().second;
    std::pair<double, double> d_rl_boundaries(2.+4.+2., 2.+4.+2.+4.); // boundaries of right-most lane (d)
    if (detn.d()>d_rl_boundaries.first & detn.d()<d_rl_boundaries.second)
      // yes, the car is on my lane! but is it the one that is next to me?
    {
      double delta_s(detn.s()-t.car_s);
      delta_s=(delta_s<-records.s_range ? delta_s+m.max_s : delta_s);
      if (delta_s>0 & delta_s<delta_s_closest)
      {
        delta_s_closest=delta_s;
        id_closest=detn.id;
      }
    }
  }

  // get (first) prediction (Predictions can contain multiple predictions for a car)
  Predictions::Prediction * pred(NULL);

  if (id_closest>=0)
    pred=predictions.preds.find(id_closest)->second;
//  // debugging
//  {
//    if (id_closest>=0)
//    {
//      std::cout<<"Next car: "<<id_closest<<", dist: "<<delta_s_closest
//              <<"m; "
//              <<"Speed from Telemetry: "
//              <<lenvec(t.sensor_fusion.at(id_closest).speed())<<"m/s; "
//              <<"Data from Predictions: "
////              <<" (mean: "
////              <<lenvec(pred->speed0)<<"m/s); Trajectory: (0s: "
////              <<m.getSmoothFrenet(pred->trajectory(0))[0]-t.car_s<<"m, "
////              <<lenvec(pred->trajectory.tangent(0))<<" m/s)"
//                ;
//    }
//  }

  // meaning see below, specification for speed planning
  const double min_dist(4.*4.);  // 4 cars
  const double max_dist(12.*4.); // 12 cars

  for (int i(1); i<=150-path_size; ++i)
  {
    double curvature(m.map_trajectory.curvature(start_s));
    double d(2.+4.+3.5);
    double rescale(1.0/(1.0+curvature*d));

    double speed(goal_speed);
    // if there is a car in my lane, check the distance an compute a max speed
    if (id_closest>=0)
    {
      double time((i-1+path_size)*0.02);
      // predict position
      double pred_s(m.getSmoothFrenet(pred->trajectory(time))[0]);
      // distance
      double dist_s(m.distance_s(start_s,pred_s));
      /* specification for speed planning: at a distance min_dist,
       * the ego speed should equal the speed of the car ahead of the ego.
       * at a distance max_dist or more, ego may drive at its goal_speed.
       * In between, the ego's speed should be interpolated "smoothly".
       */
      double scale((dist_s-min_dist)/(max_dist-min_dist));
      scale=(scale>1. ? 1. : scale); // 0<=scale<=1
      if (scale>=0)
        scale=1.-0.5*(1.+cos(M_PI*scale)); // still 0<=scale<=1, but S-shaped
      else
        scale=-scale*scale; // don't want dist_s to be smaller then min_dist!
      speed=scale*goal_speed + (1.-scale)*lenvec(pred->trajectory.tangent(time));
    }

    double s(start_s+(speed*0.02)*rescale);
    while (s>=m.max_s) s-=m.max_s;

    dvector xy(m.getSmoothXY(s, d));

    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);

    start_s=s;

//    // debugging
//    if ((pred != NULL) & ((i+path_size)%50 == 0))
//    {
//      int time((i+path_size)/50);
//      std::cout<<time<<"s: "
//              <<m.getSmoothFrenet(pred->trajectory(time))[0]-s<<"m, "
//              <<lenvec(pred->trajectory.tangent(time))<<"m/s; ";
//    }

  }
//  // debugging
//  std::cout<<std::endl;

  return r;
}


Response fe_minjerk(const Telemetry &t, const HighwayMap &m,
                                    const Records & records, const Predictions::Predictions & predictions)
{
  Response r;

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*. Here, we use
   * 25 steps from the old path. This corresponds to
   * 25*0.02s = 0.5s
   */

  int path_size = t.previous_path_x.size();
  path_size = (path_size > 25 ? 25 : path_size);
  for(int i = 0; i < path_size; i++)
  {
      r.next_x_vals.push_back(t.previous_path_x[i]);
      r.next_y_vals.push_back(t.previous_path_y[i]);
  }

  // state at the end of the pre-planned path
  dvector start_sd(path_size>0 ? m.getSmoothFrenet(t.previous_path_x[path_size-1], t.previous_path_y[path_size-1]) : dvector({t.car_s, t.car_d}));
  double start_s(start_sd[0]);
  double start_d(start_sd[1]);
  dvector v_start_speed(path_size>1
                      ? dvector({(r.next_x_vals[path_size-1]-r.next_x_vals[path_size-2])/0.02, (r.next_y_vals[path_size-1]-r.next_y_vals[path_size-2])/0.02})
                      : dvector({t.car_speed*cos(t.car_yaw), t.car_speed*sin(t.car_yaw)}));
  double start_speed(lenvec(v_start_speed));

  // goal speed (49.5 Mph)
  double goal_speed(49.5*1.609344/3.6);  // m/s

  // destination lane (0: left lane, 1: middle, 2: right lane)
  int dest_lane(2);
  // destination (desired) d
  double des_d(2.+4.+3.6*double(dest_lane-1));

  // find the car in front of us and in our lane. Is there a record of such a car?
  double delta_s_closest(1.0/0.0); // distance to the next car
  int id_closest(-1); // id of the closest car

  {
    std::pair<double, double> d_rl_boundaries(start_d-3., des_d+3.); // boundaries of lane dest_lane, including some safety distance
    for (auto r(records.records.cbegin()); r!=records.records.cend(); ++r)
    {
      Detection detn = r->second.get_last().second;
      if (detn.d()>d_rl_boundaries.first & detn.d()<d_rl_boundaries.second)
        // yes, the car is on my lane! but is it the one that is next to me?
      {
        double delta_s(HighwayMap::distance_s(t.car_s, detn.s()));
        delta_s=(delta_s<-records.s_range ? delta_s+m.max_s : delta_s);
        if (delta_s>0 & delta_s<delta_s_closest)
        {
          delta_s_closest=delta_s;
          id_closest=detn.id;
        }
      }
    }
  }

  // get (first) prediction (Predictions can contain multiple predictions for a car)
  Predictions::Prediction * pred(NULL);
  if (id_closest>=0)
    pred=predictions.preds.find(id_closest)->second;

  /* specification for acceleration planning: at a distance min_dist,
   * the ego speed should equal the speed of the car ahead of the ego.
   * at a distance max_dist or more, ego may drive at its goal_speed.
   * in both situations, acceleration should be zero.
   */
  const double min_dist(6.*4.);  // 6 cars
  const double max_dist(3.*min_dist);

  for (int i(1); i<=125-path_size; ++i)
  {
    double d;
    if (start_speed<0.01) // if the car is not moving
      d=start_d;
    else
      d=start_d+0.01*(des_d-start_d);

    double curvature(m.map_trajectory.curvature(start_s));
    double rescale(1.0/(1.0+curvature*d));

    double scale(1.);
    double des_speed(goal_speed); // desired speed
    // if there is a car in my lane, check the distance an compute a max speed
    if (id_closest>=0)
    {
      double time((i-1+path_size)*0.02);
      // predict position
      double pred_s(m.getSmoothFrenet(pred->trajectory(time))[0]);
      // distance
      double dist_s(m.distance_s(start_s,pred_s));
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
      des_speed=scale*goal_speed + (1.-scale)*lenvec(pred->trajectory.tangent(time));
      des_speed = (des_speed<0. ? 0. : des_speed);
      std::cout<<"--------- Distance: "<<dist_s<<"m, "
               <<"ego speed: "<<start_speed<<"m/s, "
               <<"desired speed: "<<des_speed<<"m/s, "
               <<"scale: "<<scale<<" ------------"<<std::endl;
    }

    double acceleration((des_speed-start_speed)/2.5
                        + (scale<0. ? 1000.*scale : 0.)); // dist<min_dist? then decellerate!
    const double max_acc(100./3.6/5.);
    acceleration = (acceleration>max_acc ? max_acc : acceleration);
    acceleration = (acceleration<-max_acc ? -max_acc : acceleration);

    double speed=start_speed+0.02*acceleration;
    speed = (speed>goal_speed ? goal_speed : speed);

    double s(start_s+(0.5*(start_speed+speed)*0.02)*rescale);
    while (s>=m.max_s) s-=m.max_s;

    dvector xy(m.getSmoothXY(s, d));

    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);

    start_s=s;
    start_d=d;
    start_speed=speed;
  }

  return r;
}

Response fe_evenmore_minjerk(const Telemetry &t, const HighwayMap &m,
                                    const Records & records, const Predictions::Predictions & predictions)
{
  Response r;

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*. Here, we use
   * 25 steps from the old path. This corresponds to
   * 25*0.02s = 0.5s
   */

  int path_size = t.previous_path_x.size();
  path_size = (path_size > 25 ? 25 : path_size);
  for(int i = 0; i < path_size; i++)
  {
      r.next_x_vals.push_back(t.previous_path_x[i]);
      r.next_y_vals.push_back(t.previous_path_y[i]);
  }

  // state at the end of the pre-planned path
  dvector start_sd(path_size>0 ? m.getSmoothFrenet(t.previous_path_x[path_size-1], t.previous_path_y[path_size-1]) : dvector({t.car_s, t.car_d}));
  double start_s(start_sd[0]);
  double start_d(start_sd[1]);
  dvector v_start_speed(path_size>1
                      ? dvector({(r.next_x_vals[path_size-1]-r.next_x_vals[path_size-2])/0.02, (r.next_y_vals[path_size-1]-r.next_y_vals[path_size-2])/0.02})
                      : dvector({t.car_speed*cos(t.car_yaw), t.car_speed*sin(t.car_yaw)}));
  double start_speed(lenvec(v_start_speed));

  // goal speed (49.5 Mph)
  double goal_speed(49.5*1.609344/3.6);  // m/s

  // destination lane (0: left lane, 1: middle, 2: right lane)
  int dest_lane(0);
  // destination (desired) d
  double des_d(2.+4.+3.6*double(dest_lane-1));

  /* specification for acceleration planning: at a distance min_dist,
   * the ego speed should equal the speed of the car ahead of the ego.
   * at a distance max_dist or more, ego may drive at its goal_speed.
   * in both situations, acceleration should be zero.
   */
  const double min_dist(6.*4.);  // 6 cars
  const double max_dist(3.*min_dist);

  int id_closest_old(-1);

  for (int i(1); i<=125-path_size; ++i)
  {
    double time((i-1+path_size)*0.02);

    // find the car in front of us and in our lane. Is there a record of such a car?
    double delta_s_closest(1.0/0.0); // distance to the next car
    int id_closest(-1); // id of the closest car
    Predictions::Prediction * pred(NULL);

    {
      std::pair<double, double> d_rl_boundaries(start_d-3., des_d+3.); // boundaries of lane dest_lane, including some safety distance
      for (auto p(predictions.preds.cbegin()); p!=predictions.preds.cend(); ++p)
      {
        Predictions::Prediction * predn_  = p->second;
        dvector pos(predn_->trajectory(time));
        dvector sd(m.getSmoothFrenet(pos));
        if (sd[1]>d_rl_boundaries.first & sd[1]<d_rl_boundaries.second)
          // yes, the car is on my lane! but is it the one that is next to me?
        {
          double delta_s(HighwayMap::distance_s(start_s, sd[0]));

          if (delta_s>0 & delta_s<delta_s_closest)
          {
            delta_s_closest=delta_s;
            id_closest=p->first;
            pred=predn_;
          }
        }
      }
    }

    double d;
    if (start_speed<0.01) // if the car is not moving
      d=start_d;
    else
      d=start_d+0.02*(des_d-start_d);

    double curvature(m.map_trajectory.curvature(start_s));
    double rescale(1.0/(1.0+curvature*d));

    double scale(1.);
    double des_speed(goal_speed); // desired speed
    // if there is a car in my lane, check the distance an compute a max speed
    if (id_closest>=0)
    {
      // predict position
      double pred_s(m.getSmoothFrenet(pred->trajectory(time))[0]);
      // distance
      double dist_s(m.distance_s(start_s,pred_s));
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
      des_speed=scale*goal_speed + (1.-scale)*lenvec(pred->trajectory.tangent(time));
      des_speed = (des_speed<0. ? 0. : des_speed);
      if (i==1 | id_closest!=id_closest_old)
        std::cout<<id_closest<<"--------- Distance: "<<dist_s<<"m, "
                 <<"ego speed: "<<start_speed<<"m/s, "
                 <<"desired speed: "<<des_speed<<"m/s, "
                 <<"scale: "<<scale<<" ------------"<<std::endl;
    }

    double acceleration((des_speed-start_speed)/2.5
                        + (scale<0. ? 1000.*scale : 0.)); // dist<min_dist? then decellerate!
    const double max_acc(100./3.6/5.);
    acceleration = (acceleration>max_acc ? max_acc : acceleration);
    acceleration = (acceleration<-max_acc ? -max_acc : acceleration);

    double speed=start_speed+0.02*acceleration;
    speed = (speed>goal_speed ? goal_speed : speed);

    double s(start_s+(0.5*(start_speed+speed)*0.02)*rescale);
    while (s>=m.max_s) s-=m.max_s;

    dvector xy(m.getSmoothXY(s, d));

    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);

    start_s=s;
    start_d=d;
    start_speed=speed;
    // debugging
    id_closest_old=id_closest;
  }

  return r;
}


