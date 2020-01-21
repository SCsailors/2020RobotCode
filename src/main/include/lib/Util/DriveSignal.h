/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
using namespace std;

class DriveSignal {
  double mLeftMotor;
  double mRightMotor;
  bool mBrakeMode;
 public:
  DriveSignal();
  DriveSignal(double left, double right);
  DriveSignal(double left, double right, bool brakeMode);

  shared_ptr<DriveSignal> NEUTRAL; 
  shared_ptr<DriveSignal> BRAKE;

  double getLeft();
  double getRight();

  bool getBrakeMode();
};
