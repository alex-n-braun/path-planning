#include "predictions.h"
#include "helpers.h"

namespace Predictions {

Straight::Straight(const Detection &detection_, const HighwayMap &hwmap, double delta_t, int steps = 12)
// steps should be a multiple of 4
  : detection(detection_), xy(steps - (steps%4) + 1), sd(steps - (steps%4) + 1)
{
  dvector pos0(detection.pos());
  xy[0] = pos0;
  dvector speed0(detection.speed());

  // initial s and d value
  dvector sd0(hwmap.getSmoothFrenet(pos0));
  sd[0] = sd0;
  // tangent vector of the street at the car's position
  dvector tangent0(hwmap.tangent(sd0[0]));

  // part of the speed vector, that is parallel to the tangent
  dvector v_speed_t(projection(tangent0, speed0));
  // part of the speed vector, that is orthogonal to the tangent
  dvector v_speed_o(diffvec(speed0, v_speed_t));
  // lengths
  double speed_t(lenvec(v_speed_t));
  double speed_o(lenvec(v_speed_o));

  for (int i(1); i<=steps; ++i)
  {
    // curvature in between the two points
    double curvature1(hwmap.curvature(sd0[0]+0.5*speed_t*delta_t));
    // value of d in between the two points
    double d1(sd0[0]+0.5*speed_o*delta_t);
    // curvature correction factor
    double rescale(1.0/(1.0+curvature1*d1));
    // update sd0
    sd0 = {sd0[0]+speed_t*delta_t*rescale, sd0[1]+speed_o*delta_t};
    sd[i]=sd0;
    // get xy
    pos0 = hwmap.getSmoothXY(sd0);
    xy[i]=pos0;
  }

  // fit trajectory splines to xy
  dvector time(5); dvector x(5); dvector y(5);
  int delta(steps/4); int index(0);
  double time0(0.);
  for (int i(0); i<5; ++i)
  {
    time[i]=time0;
    x[i]=xy[index][0];
    y[i]=xy[index][1];
    time0+=double(delta)*delta_t;
    index+=delta;
  }
  trajectory = Trajectory::Spline(time, x, y);
}

Straight::Straight(const Straight &prediction)
  : trajectory(prediction.trajectory), detection(prediction.detection),
    xy(prediction.xy), sd(prediction.sd)
{ }

}
