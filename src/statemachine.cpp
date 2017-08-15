#include "statemachine.h"
#include "ego.h"

//StateMachine::StateMachine()
//  : state(StateMachine::lanefollow) { }

StateMachine::StateMachine(StateMachine::States state_)
  : state(state_) { }

StateMachine::StateMachine(const StateMachine &sm)
  : state(sm.state) { }

