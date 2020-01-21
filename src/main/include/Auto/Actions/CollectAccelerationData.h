/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Action.h"
#include "lib/Util/CSVWriter.h"
#include "lib/Util/DriveSignal.h"
#include "lib/Physics/DriveCharacterization.h"
#include "Robot.h"

#include "frc/Timer.h"

#include <memory>
#include <vector>
#include <sstream>
#include <string>
#include <iomanip>
using namespace std;

class CollectAccelerationData : public Action {
  double kPower=.5;
  double kTotalTime=2.0;
  shared_ptr<CSVWriter> mCSVwriter;
  
  bool mTurn;
  bool mReverse;
  bool mHighGear;

  double mStartTime=0.0;
  double mPrevVelocity=0.0;
  double mPrevTime=0.0;
  double currentVelocity=0.0;
  double acceleration=0.0;
 public:
  CollectAccelerationData(vector<shared_ptr<DriveCharacterization::AccelerationDataPoint>> data, bool highGear, bool reverse, bool turn);
  vector<shared_ptr<DriveCharacterization::AccelerationDataPoint>> mAccelerationData;
  void start();
  void update();
  void done();
  bool isFinished();

  template<typename T>
  string toString(T value);
  string toCSV();
  string getFields();
};
  