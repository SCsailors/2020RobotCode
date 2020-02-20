/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <memory>

#include "Subsystem.h"
#include "Limelight.h"

#include <lib/Vision/TargetInfo.h>

#include <Constants.h>

namespace Subsystems
{
//handles multiple (limelight 2)s at the same time
class LimelightManager : public Subsystems::Subsystem {
  
  static std::shared_ptr<LimelightManager> mInstance;
  
  Subsystems::Limelight mTurretLimelight{Subsystems::LimelightConstants{"Turret Limelight", "limelight-turret", 0.0, std::make_shared<Pose2D>(0.0,0.0,0.0), std::make_shared<Rotation2D>(0.0,0.0, true)}};
  Subsystems::Limelight mIntakeLimelight{Subsystems::LimelightConstants{"Intake Limelight", "limelight-intake", 0.0, std::make_shared<Pose2D>(0.0,0.0,0.0), std::make_shared<Rotation2D>(0.0,0.0, true)}};
  std::vector<Subsystems::Limelight> mAllLimelights;
 public:
  LimelightManager();
  static std::shared_ptr<LimelightManager> getInstance();

  void OnStart(double timestamp) override;
  void OnLoop(double timestamp) override;
  void OnStop(double timestamp) override;

  void readPeriodicInputs() override;

  void writePeriodicOutputs() override;

  void outputTelemetry() override;
  void triggerOutputs();

  Subsystems::Limelight getTurretLimelight();
  Subsystems::Limelight getIntakeLimelight();
  std::vector<Subsystems::Limelight> getLimelights();

  std::vector<VisionTargeting::TargetInfo> getTargetInfos();

  double getAverageLatency();
  void setAllLEDS(Limelight::LedMode mode);
  void setPipeline(int pipeline);
};
} // namespace Subsystems
