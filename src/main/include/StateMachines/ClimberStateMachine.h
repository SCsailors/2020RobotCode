/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <States/TimedLEDState.h>

#include <Subsystems/LED.h>
#include <Subsystems/Superstructure.h>

#include <lib/Util/Util.h>

namespace StateMachines{
class ClimberState {
  Util util{};
 public:
  ClimberState(){}
  ClimberState(double winch, bool extend) : winch(winch), extend(extend) {}
  double winch = 0.0; // position inches (Position PID)
  bool extend = false; //percent out
  bool isAtDesiredState(ClimberState other)
  {
    return util.epsilonEquals(winch - other.winch, 0.0, .05);
  }
};

class ClimberStateMachine {
  Util util{};
  TimedLEDStates mLEDStates{};

 public:
  enum WantedAction {
    WANTED_PRE_CLIMB,
    WANTED_CLIMBING,
    WANTED_POST_CLIMB
  };
  
  enum SystemState {
    PRE_CLIMB,
    CLIMBING,
    POST_CLIMB
  };

  WantedAction mWantedAction = WantedAction::WANTED_POST_CLIMB;
  SystemState mSystemState = SystemState::POST_CLIMB;
  ClimberState mDesiredState{};
  std::shared_ptr<Subsystems::LED> mLED;

  double mCurrentStateStartTime = 0.0;
  bool ClimbingToPostClimb = false;
  
  ClimberStateMachine();
  WantedAction getWantedAction(){return mWantedAction;}

  SystemState getSystemState(){return mSystemState;}
  ClimberState updateClimber(double timestamp, WantedAction wantedAction, ClimberState currentState);

  SystemState defaultTransition(WantedAction wantedAction);
  
  //PRE_CLIMB
  SystemState handlePreClimbStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, ClimberState currentState);
  void getPreClimbDesiredState(ClimberState currentState, double timeInState, ClimberState &desiredState);

  //CLIMBING
  SystemState handleClimbingStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, ClimberState currentState);
  void getClimbingDesiredState(ClimberState currentState, double timeInState, ClimberState &desiredState);

  //POST_CLIMB
  SystemState handlePostClimbStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, ClimberState currentState);
  void getPostClimbDesiredState(ClimberState currentState, double timeInState, ClimberState &desiredState);
};
}