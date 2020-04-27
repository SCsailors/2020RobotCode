/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Subsystems/ServoMotorSubsystem.h>
#include <StateMachines/ClimberStateMachine.h>
#include <Constants.h>

#include <frc/Solenoid.h>

#include <memory>

namespace Subsystems {
class WinchSystem : public TalonSRXSubsystem {
  static std::shared_ptr<WinchSystem> mInstance;
  StateMachines::ClimberStateMachine mStateMachine{};
  StateMachines::ClimberState mCurrentState{};
  StateMachines::ClimberState mDesiredState{};
  StateMachines::ClimberStateMachine::WantedAction mWantedAction = StateMachines::ClimberStateMachine::WantedAction::WANTED_POST_CLIMB;

  frc::Solenoid mClimber{Constants::kPCMID, Constants::kSolenoidID_Climber};
 public:
  WinchSystem(std::shared_ptr<TalonConstants> constants);
  static std::shared_ptr<WinchSystem> getInstance();

  void OnStart(double timestamp) override;
  void OnLoop(double timestamp) override;
  void OnStop(double timestamp) override;

  void setWantedAction(StateMachines::ClimberStateMachine::WantedAction wantedAction);

  void updateCurrentState();
  void followSetpoint();

  bool mClimberExtended = false;
  void extendClimber();
  void retractClimber();
  bool isClimberExtended();
};
}