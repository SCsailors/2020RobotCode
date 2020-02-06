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

#include <frc/Solenoid.h>

#include <memory>

#include <Constants.h>

namespace Subsystems {

class WheelOfFortune : public Subsystems::TalonSRXSubsystem{
  frc::Solenoid mExtensionSolenoid{Constants::kPCMID, Constants::kSolenoidID_WheelOfFortune};
 public:
  WheelOfFortune(Subsystems::TalonConstants constants);
  
};
}