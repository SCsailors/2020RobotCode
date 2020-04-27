/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Auto/Actions/Action.h"
#include "Auto/AutoModeBase.h"

#include <memory>
using namespace std;

#include "Auto/Actions/DriveTrajectory.h"
#include "Auto/Actions/SeriesAction.h"
#include "Auto/Actions/WaitAction.h"


class StraightMode : public AutoModeBase {
  shared_ptr<DriveTrajectory> forward;
  shared_ptr<WaitAction> wait;
  shared_ptr<DriveTrajectory> reverse;
 public:
  StraightMode();
  void routine() override;
  string getID() override;
};
