/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Subsystems/ServoMotorSubsystem.h>

#include <frc/AnalogInput.h>
#include <lib/Util/LatchedBoolean.h>

#include <StateMachines/WheelStateMachine.h>

#include <frc/Solenoid.h>

#include <memory>

#include <Constants.h>

namespace Subsystems {

class WheelOfFortune : public Subsystems::TalonSRXSubsystem{
  frc::Solenoid mExtensionSolenoid{Constants::kPCMID, Constants::kSolenoidID_WheelOfFortune};
  static std::shared_ptr<WheelOfFortune> mInstance;
 
  StateMachines::WheelStateMachine mStateMachine{};
  StateMachines::WheelState mCurrentState{};
  StateMachines::WheelState mDesiredState{};
  StateMachines::WheelStateMachine::WantedAction mWantedAction = StateMachines::WheelStateMachine::WantedAction::WANTED_POST_WHEEL;
  StateMachines::WheelState::Color mCurrentColor = StateMachines::WheelState::Color::NONE;
  StateMachines::WheelState::Color mDesiredColor = StateMachines::WheelState::Color::NONE;

 public:
  WheelOfFortune(std::shared_ptr<Subsystems::TalonConstants> constants);
  static std::shared_ptr<WheelOfFortune> getInstance();

  void OnStart(double timestamp) override;
  void OnLoop(double timestamp) override;
  void OnStop(double timestamp) override;

  void setWantedAction(StateMachines::WheelStateMachine::WantedAction wantedAction);
  void updateCurrentState();
  void followSetpoint();

  bool mWheelExtended = false;
  void extendWheel();
  void retractWheel();
  bool isWheelExtended();

  void setColorGoal(StateMachines::WheelState::Color wantedColor);
  StateMachines::WheelState::Color getColorGoal();
  
};
}