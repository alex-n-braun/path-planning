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

  enum States { lanefollow, lanechange };
  States state;
//  StateMachine();
  StateMachine(States state_);
  StateMachine(const StateMachine & sm);
};

#endif // STATEMACHINE_H
