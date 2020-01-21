/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/AutoModeBase.h"
#include "Auto/Actions/PIDTuner.h"
#include "Auto/Actions/SeriesAction.h"
#include "Auto/Actions/Action.h"

#include <memory>
#include <vector>
using namespace std;

class PIDTuningMode : public AutoModeBase{
  //if using this mode, set PIDTuning in Drive.h to true, otherwise it's false.
  
  vector<double> setpoints{1.0, .3, .7}; 
  
 public:
  PIDTuningMode();
  void routine() override;
  string getID() override;
  vector<shared_ptr<Action>> createProfile(vector<double> percentOutput);

};
