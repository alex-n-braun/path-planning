#include "statemachine.h"
#include "ego.h"

StateMachine::StateMachine()
  : state(StateMachine::lanefollow)
{

}

bool StateMachine::transition(const Status &status)
{
  bool update(false);

  switch (state)
  {
  case StateMachine::lanefollow:
    {

    }
    break;

  }

  return update;
}

