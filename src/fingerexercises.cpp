#include "fingerexercises.h"
#include "helpers.h"
#include "trajectory.h"
#include "models.h"

//constexpr double pi() { return M_PI; }
//static double deg2rad(double x) { return x * pi() / 180; }


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

  int index(m.ClosestWaypoint(t.car_x, t.car_y));

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

    if (i==1)
      std::cout<<index<<", "<<t.car_x<<", "<<t.car_y<<", "<<t.car_s<<", "<<t.car_speed<<", curve: "<<curvature<<", rescale: "<<rescale<<std::endl;
  }

  return r;
}
