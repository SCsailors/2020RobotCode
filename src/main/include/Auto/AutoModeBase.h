/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Actions/Action.h"

#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <memory>
#include <iostream>
using namespace std;


class AutoModeBase {
  frc::Timer timer;
  double mUpdateRate=1.0/50.0;
  bool mActive=false;
  
 public:
 double timestamp=0.0;
 double prev_timestamp=0.0;
  AutoModeBase();
  virtual void routine(){}
  void run();
  virtual void done();
  virtual bool isActive();
  virtual string getID();
  void setActive(bool run);
  void runAction(shared_ptr<Action> action);
};
