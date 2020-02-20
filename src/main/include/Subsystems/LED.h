/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Subsystems/Subsystem.h"

#include <memory>
#include <iostream>

#include <frc/Spark.h>

#include "States/TimedLEDState.h"

#include <lib/Util/Util.h>
namespace Subsystems{
class LED : public Subsystems::Subsystem {
  Util util{};
  TimedLEDStates mLEDStates{};
  static std::shared_ptr<LED> mLEDInstance;
 public:
  enum WantedAction {
      DISPLAY_START, 
      DISPLAY_FAULT, 
      DISPLAY_HANG, 
      DISPLAY_INTAKING, 
      DISPLAY_SHOOTING, 
      DISPLAY_ZEROING, 
      DISPLAY_BALLS, 
      DISPLAY_WHEEL,
      DISPLAY_END
    };
  enum SystemState {
      DISPLAYING_START, 
      DISPLAYING_FAULT, 
      DISPLAYING_HANG, 
      DISPLAYING_INTAKING, 
      DISPLAYING_SHOOTING, 
      DISPLAYING_ZEROING, 
      DISPLAYING_BALLS, 
      DISPLAYING_WHEEL,
      DISPLAYING_END
    };

  SystemState mSystemState = SystemState::DISPLAYING_START;
  WantedAction mWantedAction = WantedAction::DISPLAY_START;

  LEDState mDesiredLEDState{-.99};
  
  std::shared_ptr<TimedLEDState> mStartEndLEDState = mLEDStates.kIdle;
  std::shared_ptr<TimedLEDState> mHangLEDState = mLEDStates.kStaticOff;
  std::shared_ptr<TimedLEDState> mIntakeLEDState = mLEDStates.kStaticOff;
  std::shared_ptr<TimedLEDState> mShootingLEDState = mLEDStates.kStaticOff;
  std::shared_ptr<TimedLEDState> mBallsLEDState = mLEDStates.kStaticOff;
  std::shared_ptr<TimedLEDState> mWheelLEDState = mLEDStates.kStaticOff;
  std::shared_ptr<TimedLEDState> mFaultLEDState = mLEDStates.kFault;
  std::shared_ptr<TimedLEDState> mJustZeroedLEDState = mLEDStates.kJustZeroed;
  std::shared_ptr<TimedLEDState> mRobotZeroedLEDState = mLEDStates.kRobotZeroed;
  std::shared_ptr<TimedLEDState> mStaticOff = mLEDStates.kStaticOff;
  //TimedLEDState mEndLEDState = TimedLEDStates::kStaticOff;

  bool mFault = false;
  double stateStartTime = 0.0;
  double mLastZeroTime = NAN;

  LED(){}
  frc::Spark blinkinLEDIntake{0}; //set for actual robot
  frc::Spark blinkinLEDShooter{1};

  void setFault(){mFault = true;}
  void clearFault(){mFault = false;}

  static std::shared_ptr<LED> getInstance();

  void setHangLEDState(std::shared_ptr<TimedLEDState> hangLEDState){mHangLEDState = hangLEDState;}
  void setIntakeLEDState(std::shared_ptr<TimedLEDState> intakeLEDState){mIntakeLEDState = intakeLEDState;}
  void setShootLEDState(std::shared_ptr<TimedLEDState> shootLEDState){mShootingLEDState = shootLEDState;}
  void setBallLEDState(std::shared_ptr<TimedLEDState> ballLEDState){mBallsLEDState = ballLEDState;}
  void setWheelLEDState(std::shared_ptr<TimedLEDState> wheelLEDState){mWheelLEDState = wheelLEDState;}
  
  void setWantedAction(WantedAction wantedAction){mWantedAction = wantedAction;}

  void OnStart(double timestamp) override;

  void OnLoop(double timestamp) override;
  
  void OnStop(double timestamp) override;

  void updateZeroed(){mLastZeroTime = frc::Timer::GetFPGATimestamp();}

  void setStartLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setFaultLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setHangLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setIntakingLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setShootingLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setZeroingLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setBallsLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setWheelLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setEndLEDCommand(LEDState &desiredLEDState, double timeInState); 
  void setOffLEDCommand(LEDState &desiredLEDState, double timeInState); 

  SystemState getStateTransition();
  

  
};
}