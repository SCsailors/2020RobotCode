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


#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <vector>
#include <memory>

#include <sstream>
#include <string>
#include <iomanip>
using namespace std;



class CollectVelocityData : public Action{
  double pi=3.14159265;
  double kMaxPower=.25;
  double kRampRate=.02;
  double mPercentPower=0.0;
  int i=0;

  double velocity=0.0;
  double voltage=0.0;
  shared_ptr<DriveSignal> signal;
  shared_ptr<CSVWriter> mCSVWriter;
  
  bool mTurn;
  bool mReverse;
  bool mHighGear;

  bool Finished=false;
  double mStartTime=0.0;
 public:
  
  CollectVelocityData(vector<shared_ptr<DriveCharacterization::VelocityDataPoint>> data, bool highgear, bool reverse, bool turn);
  vector<shared_ptr<DriveCharacterization::VelocityDataPoint>> mVelocityData;

  bool isFinished();
  void start();
  void update();
  void done();
  string getFields();
  template< typename T>
  string toString(T value);
  string toCSV();
};
  