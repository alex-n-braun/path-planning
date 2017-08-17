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
  int dest_lane;
  States state;
//  StateMachine();
  StateMachine(States state_, int dest_lane_);
  StateMachine(const StateMachine & sm);
};

#endif // STATEMACHINE_H
