/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Subsystems/Subsystem.h"

#include "frc/Timer.h"
#include "frc/Notifier.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <vector>
#include <memory>
#include <iostream>
using namespace std;

class Looper {
  
  double UpdateRate= .005;
  vector<shared_ptr<Subsystems::Subsystem>> mSubsystems;
  bool running_=false;
  bool disabled_running_=false;
  double timestamp=0.0;
  //creates a thread that updates subsystems and other parts of the code at 200Hz (5ms)
  
  //will be used in the future for trajectory generation
  //std::function<void()> enabledLoops= []() {mLoops.onLoop();};
  //frc::Notifier EnabledLoops{enabledLoops}; 
 public:
 //shared_ptr<Subsystem> mSubsystems;
  Looper(vector<shared_ptr<Subsystems::Subsystem>> subsystem);
  Looper();
  void writeToLog();
  void outputToSmartDashboard();
  void OnStart(double timestamp);
  void OnLoop();
  void OnStop(double timestamp);
  void resetSensors();

  void startEnabledLoops();
  void stopEnabledLoops();
  void startDisabledLoops();
  void stopDisabledLoops();

 
  std::function<void()> Test1 = [&]() {OnLoop();};
  frc::Notifier mEnabledLoops{Test1};
};
