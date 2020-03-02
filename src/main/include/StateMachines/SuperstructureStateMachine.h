/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include "States/SuperstructureState.h"
#include "States/SuperstructureGoal.h"
#include "lib/Util/TimeDelayedBoolean.h"
#include "lib/Util/LatchedBoolean.h"
#include "lib/Util/VariableDelayedBoolean.h"

#include <lib/Util/Util.h>

#include "Subsystems/LED.h"

namespace StateMachines{
class SuperstructureStateMachine {
  Util util{};
  TimedLEDStates mLEDStates{};
  bool IntakingBallTOHaveBallTransition = false;

  bool changedIntake = false;
  bool changedWheelie = false;
  bool shooterPriority = false;
  bool intakePriority = false;

  bool preBottomState = false;
  bool topPathState = false;
  bool bottomPathState = false;

  bool topPathTriggered = false;
  bool bottomPathTriggered = false;
  bool preBottomTriggered = false;
  bool ballPathforward = true;
  bool runBallPath = false;

  bool haveBallsAdjFinished = false;
  Utility::TimeDelayedBoolean topBallTrigger{};
  Utility::TimeDelayedBoolean bottomBallTrigger{};
  Utility::VariableDelayedBoolean PreBottomTrigger{};

 public:
  enum WantedAction {
        WANTED_IDLE, 
        WANTED_PRE_EXHAUST_BALL, 
        WANTED_EXHAUST_BALL, 
        WANTED_INTAKE_BALL, 
        WANTED_HAVE_BALLS};
        
  enum SystemState {
        IDLE, 
        INTAKING_BALL, 
        PRE_EXHAUSTING_BALL, 
        EXHAUSTING_BALL, 
        HAVE_BALLS};

  enum Range {CLOSE, MID, FAR};
 
  SuperstructureStateMachine();
  
  SystemState mSystemShooterState = SystemState::IDLE;
  SystemState mSystemIntakeState = SystemState::IDLE;
  Range mRange = Range::CLOSE;
  double mRangeInches = 0.0;
  SuperstructureState mDesiredShooterState{};
  SuperstructureState mDesiredIntakeState{};
  SuperstructureGoal mDesiredGoal{};
  std::shared_ptr<Subsystems::LED> mLED;
  double mCurrentStateStartTimeShooter = 0.0;
  double mCurrentStateStartTimeIntake = 0.0;

  double kMaxIntaking = 30.0; // TUNE
  double kMinIntaking = 2.0;

  double ballgoal = 0.0;
  bool doneShooting = false;
  bool preExhausting = false;

  SystemState getSystemShooterState(){return mSystemShooterState;}
  SystemState getSystemIntakeState(){return mSystemIntakeState;}
  SuperstructureState updateShooting(double timestamp, WantedAction wantedAction, SuperstructureState currentState, double range);
  SuperstructureState updateIntaking(double timestamp, WantedAction wantedAction, SuperstructureState currentState);

  SuperstructureGoal mergedShootingIntaking(double timestamp, WantedAction wantedActionShooter, WantedAction wantedActionIntake, SuperstructureState currentState, double range);
  double resolveState(double shooterState, double intakeState, double currentState);
  bool resolveState(bool s1, bool s2);

  SuperstructureStateMachine::SystemState defaultTransition(WantedAction wantedAction);
  
  //IDLE
  SuperstructureStateMachine::SystemState handleIdleStateTransitions(WantedAction wantedAction);
  void getIdleDesiredState(SuperstructureState currentState, double timeInState, SuperstructureState &desiredState, bool shooter);

  //INTAKING_BALL
  SuperstructureStateMachine::SystemState handleIntakingBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction);
  void getIntakingBallDesiredState(SuperstructureState currentState, SuperstructureState &desiredState);

  //PRE_EXHAUSTING_BALL
  SuperstructureStateMachine::SystemState handlePreExhaustingBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction);
  void getPreExhaustingBallDesiredState(SuperstructureState currentState, double timeInState, SuperstructureState &desiredState);

  //EXHAUSTING_BALL
  SuperstructureStateMachine::SystemState handleExhaustingBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction);
  void getExhaustingBallDesiredState(SuperstructureState currentState, SuperstructureState &desiredState);
  
  //HAVE_BALL
  SuperstructureStateMachine::SystemState handleHaveBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction);
  void getHaveBallDesiredState(SuperstructureState currentState, double timeInState, SuperstructureState &desiredState);

  void setBallGoal(double balls){ballgoal = balls;}
  double getBallGoal(){return ballgoal;}
  
  void setRange(Range range){mRange = range;};
  Range getRange(){return mRange;};

  bool mLEDPriority = false;
  void setLEDPriority(bool priority){mLEDPriority = priority;}
  bool mLEDMaxPriority = false;
  void setLEDMaxPriority(bool maxPriority){mLEDMaxPriority = maxPriority;}
  
  void resetIntakeLogic();
  void updateBottomPathState(bool state){bottomPathState = state;}
  void updateTopPathState(bool state){topPathState = state;}
  void updatePreBottomPathState(bool state){preBottomState = state;}
  //add double getHoodForRange(range);
  //add double getSpeedForRange(range);
  //use either piecewise functions or multiple interpolating treemap (one for each speed) where all the data is already inputted.

};
}