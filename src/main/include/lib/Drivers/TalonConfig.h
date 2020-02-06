/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
//#include "ctre/phoenix/motorcontrol/can/TalonFX.h" : example to include specific file
namespace Drivers{

class TalonConfig {
 public:
  TalonConfig();
  double NEUTRAL_DEADBAND = 0.04;

  bool INVERTED = false;
  bool SENSOR_PHASE = false;

  int CONTROL_FRAME_PERIOD_MS = 5;
  int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
  int GENERAL_STATUS_FRAME_RATE_MS = 5;
  int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
  int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
  int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
  int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

  ctre::phoenix::motorcontrol::VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_100Ms;
  int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

  std::shared_ptr<TalonConfig> kDefaultConfig;
  std::shared_ptr<TalonConfig> kSlaveConfig;


};
}