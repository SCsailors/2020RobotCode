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
const static double kShooterPaddingRPS = 100.0;
const static double kHoodPaddingDegrees = 5.0;
const static double kBallPathTopPaddingRPS = 100.0;
const static double kBallPathBottomPaddingRPS = 100.0;
const static double kCenteringIntakePaddingRPS = 100.0;
const static std::vector<double> kPadding{
    kTurretPaddingDegrees, kShooterPaddingRPS, kHoodPaddingDegrees, kBallPathTopPaddingRPS, kBallPathBottomPaddingRPS, kCenteringIntakePaddingRPS};
}