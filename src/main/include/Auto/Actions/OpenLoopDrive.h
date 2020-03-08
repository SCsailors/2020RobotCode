/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Action.h"
#include "lib/Util/DriveSignal.h"

#include "frc/Timer.h"

#include <memory>
using namespace std;

class OpenLoopDrive: public Action {
  //double mStartTime;
  double mDuration, mLeft, mRight = 0.0;
  bool mFinishCondition = false;
  frc::Timer mTimer{};
  
 public:
  OpenLoopDrive(double left, double right, double duration, bool finishCondition);
  void start();
  void update();
  void done();
  bool isFinished();
};
  