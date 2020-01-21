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
#include <vector>
using namespace std;

class PIDTuner : public Action{
  double mStartTime=0.0;
  double mDuration=0.0;
  double mVelocity=0.0;
  double mFeedForward=0.0;
  double mEndFeedForward=0.0;
  double mEndVelocity=0.0;
  double mVelocityIPS=0.0;
  double maxNeoRPM=4500.0;
  double kPIDVoltage=10.0;
  //actual max 5676 according to data sheet, but 5500 is highest recorded.
  shared_ptr<DriveSignal> output;
  shared_ptr<DriveSignal> feedForward;
  
 public:
  PIDTuner(double percentPower, double duration);
  PIDTuner(double percentPower, double endPower, double duration);
  void start();
  void update();
  void done();
  bool isFinished();
  
};
