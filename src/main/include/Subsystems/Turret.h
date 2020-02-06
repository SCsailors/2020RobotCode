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

class Turret : public Subsystems::SparkMaxSubsystem {
  static std::shared_ptr<Subsystems::Turret> mInstance;
  frc::AnalogInput mBannerInput{2}; // TODO: change to reflect actual
  Utility::LatchedBoolean mJustReset{};
  bool mHoming = false;
  bool kUseManualHomingRoutine = false;
 public:
  Turret(Subsystems::SparkMaxConstants constants);
  
  Turret(){};
  static std::shared_ptr<Subsystems::Turret> getInstance();
  double getAngle();
  bool atHomingLocation();
  void handleMasterReset(bool reset) override;
  bool isHoming();
  
};
}