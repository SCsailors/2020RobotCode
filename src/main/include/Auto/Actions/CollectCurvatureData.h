/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Actions/Action.h"
#include "lib/Util/CSVWriter.h"
#include "lib/Physics/DriveCharacterization.h"
#include "lib/Util/DriveSignal.h"

#include "frc/Timer.h"

#include <memory>
#include <vector>
#include <sstream>
#include <string>
#include <iomanip>
using namespace std;

class CollectCurvatureData : public Action {
  double kMaxPower=.4;
  double kStartPower= .2;
  double kStartTime= .25;
  double kRampRate= .02;
  bool mReverse;
  bool mHighGear;
  bool Finished= false;
  double mStartTime=0.0;
  double rightPower=0.0;
  vector<shared_ptr<DriveCharacterization::CurvatureDataPoint>> mCurvatureData;
  shared_ptr<CSVWriter> mCSVWriter;
 public:
  CollectCurvatureData(vector<shared_ptr<DriveCharacterization::CurvatureDataPoint>> data, bool highGear, bool reverse);
  void start();
  void update();
  void done();
  bool isFinished();

  string toCSV();
  template<typename T>
  string toString(T value);
  string getFields();

};
  