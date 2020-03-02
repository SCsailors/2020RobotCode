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
class CenteringIntake : public Subsystems::SparkMaxSubsystem {
 static std::shared_ptr<Subsystems::CenteringIntake> mInstance;
 public:
  CenteringIntake(std::shared_ptr<Subsystems::SparkMaxConstants> constants);
  static std::shared_ptr<Subsystems::CenteringIntake> getInstance();
};
}