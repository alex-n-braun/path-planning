#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "trajectory.h"
#include "predictions.h"

class StateMachine
{
public:
  enum States { maxspeed, constdist };
  States state;
  StateMachine();
  bool transition(const Trajectory::MinJerk * trajectory,
                  const Predictions::Predictions & predictions, double time);
};

#endif // STATEMACHINE_H
