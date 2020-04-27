/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Action.h"

#include <vector>
#include <memory>
using namespace std;



class SeriesAction : public Action {
 vector<shared_ptr<Action>> mRemainingActions;
 shared_ptr<Action> mCurrAction;
 public:
  SeriesAction(vector<shared_ptr<Action>> actions);
  bool isFinished();
  void start();
  void update();
  void done();
};
  