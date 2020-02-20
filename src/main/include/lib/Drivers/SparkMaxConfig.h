/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <rev/CANSparkMax.h>
#include <memory>
namespace Drivers{
class SparkMaxConfig {
 public:
  SparkMaxConfig();
  bool BURN_FACTORY_DEFAULT_FLASH = false;
  rev::CANSparkMax::IdleMode NEUTRAL_MODE = rev::CANSparkMax::IdleMode::kCoast;

  int STATUS_FRAME_0_RATE_MS = 10;
  int STATUS_FRAME_1_RATE_MS = 1000;
  int STATUS_FRAME_2_RATE_MS = 1000;
};

class SparkMaxConfigBase {
 public:
  SparkMaxConfigBase();
  std::shared_ptr<SparkMaxConfig> kDefaultConfig;
  std::shared_ptr<SparkMaxConfig> kSlaveConfig;
};
}