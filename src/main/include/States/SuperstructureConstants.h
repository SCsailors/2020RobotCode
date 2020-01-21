/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
namespace SuperstructureConstants{
const static double kTurretPaddingDegrees = 5.0;
const static double kShooterPaddingRPM = 100.0;
const static double kWristPaddingDegrees = 5.0;
const static std::vector<double> kPadding{
    kTurretPaddingDegrees, kShooterPaddingRPM, kWristPaddingDegrees};
}