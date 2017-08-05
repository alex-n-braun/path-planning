#include "ego.h"

Ego::Ego()
  : min_jerk(NULL), init(0)
{
  re_init();
}

Ego::~Ego()
{
  delete min_jerk;
}

void Ego::re_init()
{
  init=1;
}

Response Ego::path(const Telemetry &t, const HighwayMap &m, const Records &records, const Predictions::Predictions &predictions)
{
  Response r;

  const double dt(0.02);

  /* we have computed a path in the previous computation step.
   * This information can be reused. The simulator sends the
   * not-yet executed steps in previous_path_*.
   */

  int path_size = t.previous_path_x.size();
  int keep_steps(25); // (init==0 ? 50 : 0);
  int max_path_size(200); //(init==0 | path_size==0 ? 200 : path_size);

  int consumed_steps(max_path_size-path_size);
  std::cout<<init<<": "<<consumed_steps<<", "<<path_size<<std::endl;

  path_size = (path_size > keep_steps ? keep_steps : path_size);
  keep_steps = path_size;

  for(int i = 0; i < path_size; i++)
  {
      r.next_x_vals.push_back(t.previous_path_x[i]);
      r.next_y_vals.push_back(t.previous_path_y[i]);
  }

  // start state
  double start_s, start_speed_s, start_accel_s,
         start_d, start_speed_d, start_accel_d;
  if (init>0)
  {
    dvector start_sd({t.car_s, t.car_d});
    start_s = start_sd[0];
    start_d = start_sd[1];
    dvector v_start_speed({t.car_speed*cos(t.car_yaw), t.car_speed*sin(t.car_yaw)});
    // speed in tangential direction of lane
    double start_speed_t = projectionlength(m.tangent(start_s), v_start_speed);
    double curvature(m.map_trajectory.curvature(start_s));
    double start_rescale(1.0/(1.0+curvature*start_d));
    start_speed_s = start_speed_t * start_rescale;
    // orthogonal component
    start_speed_d = projectionlength(m.orthogonal(start_s), v_start_speed);
    // accelerations
    start_speed_s=0; start_speed_d=0;
    start_accel_s = 0.;
    start_accel_d = 0.;
  }
  else
  {
    dvector sdfull(min_jerk->sd_full(double(consumed_steps)*dt));
    start_s = sdfull[0]; start_speed_s = sdfull[1]; start_accel_s = sdfull[2];
//    std::cout<<"--------"<<start_s<<"---------"<<std::endl;
    start_d = sdfull[3]; start_speed_d = sdfull[4]; start_accel_d = sdfull[5];
  }


  // goal speed (49.5 Mph)
  double goal_speed(49.5*1.609344/3.6);  // m/s

  // destination lane (0: left lane, 1: middle, 2: right lane)
  int dest_lane(0);
  // destination (desired) d
  double des_d(2.+4.+3.6*double(dest_lane-1));

  // speed in s direction
  // double goal_rescale(1.0/(1.0+curvature*des_d));
  double goal_speed_s(goal_speed);

//  if (init>3)
//  {
//    des_d=start_d;
//    goal_speed_s=0;
//  }

  // jerk-minimizing trajectory with unspecified final s
//  if (path_size==0 | init==0)
  {
    delete min_jerk;
    min_jerk = new Trajectory::MinJerk(Trajectory::TimeRange(0., 4.5),
                                 Trajectory::VecRange({start_s, start_speed_s, start_accel_s},
                                          {goal_speed_s, 0., 0.}),
                                 Trajectory::VecRange({start_d, start_speed_d, start_accel_d},
                                          {des_d, 0., 0.}), m, 1);
  }

  for (int i(1); i<=max_path_size-path_size; ++i)
  {
    double delta_time(double(i)*dt);
    dvector xy((*min_jerk)(delta_time));
    r.next_x_vals.push_back(xy[0]);
    r.next_y_vals.push_back(xy[1]);
  }

  init--; init = (init<0 ? 0 : init);

  return r;
}

