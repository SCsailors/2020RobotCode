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
class WheelState {
  
 public:
    // in order top to the right: blue, green, red, yellow
  enum Color {
    NONE,
    BLUE,
    GREEN,
    RED,
    YELLOW
  };
  WheelState(){}
  WheelState(Color color, bool extend, double wheel, Color goal, int count) : color(color), extendWheel(extend), wheelDemand(wheel), colorGoal(goal), colorCount(count) {}
  Color color = Color::NONE; //current Color
  bool extendWheel = false; 
  double wheelDemand = 0.0; //percent out? //Position? //velocity?
  Color colorGoal = Color::NONE; //wanted end color
  int colorCount = 0; //# of same color to pass
};

class WheelStateMachine {
  TimedLEDStates mLEDStates{};
  Util util{};
 public:
  enum WantedAction {
    WANTED_PRE_WHEEL,
    WANTED_ROTATION,
    WANTED_POSITION,
    WANTED_POST_WHEEL
  };
  enum SystemState {
    PRE_WHEEL,
    ROTATION,
    POSITION,
    POST_WHEEL
  };

  WantedAction mWantedAction = WantedAction::WANTED_POST_WHEEL;
  SystemState mSystemState = SystemState::POST_WHEEL;
  WheelState mDesiredState{};
  std::shared_ptr<Subsystems::LED> mLED;

  double mCurrentStateStartTime = 0.0;
  bool RotPosToPostClimb = false; //set if Rotation or Position complete: flash completed otherwise skip

  WheelStateMachine();
  WantedAction getWantedAction(){return mWantedAction;}
  SystemState getSystemState(){return mSystemState;}
  
  WheelState updateWheel(double timestamp, WantedAction wantedAction, WheelState currentState);

  SystemState defaultTransition(WantedAction wantedAction);

  SystemState handlePreWheelStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState);
  void getPreWheelDesiredState(WheelState currentState, double timeInState, WheelState &desiredState);
  
  SystemState handleRotationStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState);
  void getRotationDesiredState(WheelState currentState, double timeInState, WheelState &desiredState);

  SystemState handlePositionStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState);
  void getPositionDesiredState(WheelState currentState, double timeInState, WheelState &desiredState);

  SystemState handlePostWheelStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState);
  void getPostWheelDesiredState(WheelState currentState, double timeInState, WheelState &desiredState);
};
}