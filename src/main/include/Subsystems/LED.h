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

#include "States/LEDState.h"
#include "States/TimedLEDState.h"
namespace Subsystems{
class LED : public Subsystems::Subsystem {
  
  //LED(LED const&){}
  //LED& operator = (LED const&){}
  static std::shared_ptr<LED> mLEDInstance;

  enum WantedAction { DISPLAY_FAULT, DISPLAY_HANG, DISPLAY_INTAKING, DISPLAY_SHOOTING, DISPLAY_ZEROING};
  enum SystemState { DISPLAYING_FAULT, DISPLAYING_HANG, DISPLAYING_INTAKING, DISPLAYING_SHOOTING, DISPLAYING_ZEROING};

  SystemState mSystemState = SystemState::DISPLAYING_INTAKING;
  WantedAction mWantedAction = WantedAction::DISPLAY_INTAKING;
  LEDState mDesiredLEDState{.99};
  TimedLEDState mIntakeLEDState; // TODO: add states once we know.
  bool mFault = false;
  double stateStartTime = 0.0;

 public:
  LED(){}
  frc::Spark blinkinLED{0}; //set for actual robot
  
  void setFault(){mFault = true;}
  void clearFault(){mFault = false;}

  static std::shared_ptr<LED> getInstance();



  void setIntakeLEDState(TimedLEDState intakeLEDState){mIntakeLEDState = intakeLEDState;}

//  void setClimbLEDState(TimedLEDState climbLEDState);
  
//  void setShootLEDState(TimedLEDState shootLEDState);

//  void setSpinnyLEDState(TimedLEDState spinnyLEDState);

  void setWantedAction(WantedAction wantedAction){mWantedAction = wantedAction;}

  void OnStart(double timestamp) override;

  void OnLoop(double timestamp) override;
  
  void OnStop(double timestamp) override;

  void updateZeroed();

  void setIntakeLEDCommand(double timeInState);

  SystemState getStateTransition();

  
};
}