#include "statemachine.h"

StateMachine::StateMachine()
  : state(maxspeed)
{

}

bool StateMachine::transition(const Trajectory::MinJerk *trajectory, const Predictions::Predictions &predictions, double time)
{
  return false;
}

