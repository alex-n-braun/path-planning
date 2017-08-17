#include "statemachine.h"
#include "ego.h"

//StateMachine::StateMachine()
//  : state(StateMachine::lanefollow) { }

StateMachine::StateMachine(StateMachine::States state_, int dest_lane_)
  : state(state_), dest_lane(dest_lane_) { }

StateMachine::StateMachine(const StateMachine &sm)
  : state(sm.state), dest_lane(sm.dest_lane) { }

