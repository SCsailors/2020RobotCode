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

#include <memory>

namespace Subsystems{

class Shooter : public Subsystems::TalonFXSubsystem {
  static std::shared_ptr<Subsystems::Shooter> mInstance;
 public:
  Shooter(std::shared_ptr<Subsystems::TalonConstants> constants);
  static std::shared_ptr<Subsystems::Shooter> getInstance();
};
}