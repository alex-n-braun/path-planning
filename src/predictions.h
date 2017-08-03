#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include "detection.h"
#include "highwaymap.h"
#include "trajectory.h"
#include "helpers.h"

namespace Predictions
{

/* the straight class not just predicts
 * a movement with constant velocity in direction
 * of the velocity vector, but also considers the
 * curvature of the road. If, for example, the velocity
 * vector of the detection is parallel to the
 * tangential vector of the street, the Straight
 * prediction predicts that the car will follow the lane.
 */
class Straight {
public:
  std::vector<dvector> xy;
  std::vector<dvector> sd;
  Trajectory::Spline trajectory;
  const Detection & detection;
  Straight(const Detection & detection_, const HighwayMap & hwmap, double delta_t, int steps);
  Straight(const Straight & prediction);
};

}

#endif // PREDICTIONS_H
