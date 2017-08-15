#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include "detection.h"
#include "highwaymap.h"
#include "trajectory.h"
#include "records.h"
#include "helpers.h"

namespace Predictions
{

/* base class for a prediction model
 */

class Prediction {
public:
  std::vector<dvector> xy;
  std::vector<dvector> sd;
  Trajectory::Spline trajectory;
  dvector speed0;
  const Record & record;
  Prediction(const Record & record_, double delta_t, int steps);
  Prediction(const Prediction & prediction);
};

/* the straight class not just predicts
 * a movement with constant velocity in direction
 * of the velocity vector, but also considers the
 * curvature of the road. If, for example, the velocity
 * vector of the detection is parallel to the
 * tangential vector of the street, the Straight
 * prediction predicts that the car will follow the lane.
 */
class Straight : public Prediction {
public:
  Straight(const Record & record_, const HighwayMap & hwmap, double delta_t, int steps);
  Straight(const Straight & prediction);
};

class Predictions {
public:
  std::multimap<int, Prediction*> preds;
  typedef std::pair<int, Prediction*> ID_Prediction;
  Predictions(const Records & records, const HighwayMap &hwmap, double delta_t, int steps = 12);
  ~Predictions();
};

}

#endif // PREDICTIONS_H
