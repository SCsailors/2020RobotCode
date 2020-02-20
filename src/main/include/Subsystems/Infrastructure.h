/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Subsystems/Subsystem.h"
#include "Constants.h"

#include <memory>

#include <frc/Compressor.h>

namespace Subsystems{

//compressor never runs while shooting
class Infrastructure : public Subsystems::Subsystem {
  static std::shared_ptr<Subsystems::Infrastructure> mInstance;
  bool manualControl = false;
 public:
  Infrastructure();
  static std::shared_ptr<Subsystems::Infrastructure> getInstance();

  void OnStart(double timestamp) override;
  void OnLoop(double timestamp) override;
  void OnStop(double timestamp) override;

  bool isManualControl();
  void setManualControl(bool manual);

  void startCompressor();
  void stopCompressor();

  frc::Compressor compressor{Constants::kPCMID};
};
}