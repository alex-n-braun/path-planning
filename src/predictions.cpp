#include "predictions.h"
#include "helpers.h"
#include <iostream>

namespace Predictions {

Prediction::Prediction(const Record &record_, double delta_t, int steps = 12)
// steps should be a multiple of 4
  : record(record_), xy(steps - (steps%4) + 1), sd(steps - (steps%4) + 1), speed0({0.,0.})
{ }

Prediction::Prediction(const Prediction &prediction)
  : trajectory(prediction.trajectory), record(prediction.record),
    xy(prediction.xy), sd(prediction.sd), speed0(prediction.speed0)
{ }

Straight::Straight(const Record &record_, const HighwayMap &hwmap, double delta_t, int steps = 12)
// steps should be a multiple of 4
//  : record(record_), xy(steps - (steps%4) + 1), sd(steps - (steps%4) + 1)
  : Prediction(record_, delta_t, steps)
{
  auto last_record_entry(record.get_last());
  Detection detection(last_record_entry.second);
  double last_time_stamp(last_record_entry.first);
  dvector pos0(detection.pos());
  xy[0] = pos0;

  // compute the speed from the last N detections, if available
  if (record.count()>=12)
  {
    double totweight(0.);;
    for (int i(record.size()-1); i>=record.size()-12; --i)
    {
      double weight(exp((record[i].first-last_time_stamp)/0.125));
      speed0 = sumvec(speed0, scalevec(record[i].second.speed(), weight));
      totweight += weight;
    }
    speed0 = scalevec(speed0, 1./totweight);
//    std::cout<<"observed speed: "<<lenvec(speed0)<<std::endl;
  }
  else
    speed0 = detection.speed();

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
    double d1(sd0[1]+0.5*speed_o*delta_t);
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
  : Prediction(prediction)
//  : trajectory(prediction.trajectory), record(prediction.record),
  //    xy(prediction.xy), sd(prediction.sd)
{ }


// generate predictions for each car in the records
// currently only the Straight prediction exists
Predictions::Predictions(const Records &records, const HighwayMap &hwmap, double delta_t, int steps)
{
  for (auto r(records.records.cbegin()); r!=records.records.cend(); ++r)
  {
    preds.insert(std::pair<int, Prediction*>(r->first, new Straight(r->second, hwmap, delta_t, steps)));
  }
}

Predictions::~Predictions()
{
  for (auto p(preds.cbegin()); p!=preds.cend(); ++p)
  {
    delete p->second;
  }
}

}
