#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "trajectory.h"
#include "predictions.h"
#include "highwaymap.h"

class StateMachine
{
public:
  class Status {
  public:
  };

  enum States { lanefollow };
  States state;
  StateMachine();
  bool transition(const Status &status);
};

#endif // STATEMACHINE_H
