/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Action.h"

#include <memory>
#include <vector>
using namespace std;



class ParallelAction :public Action {
  vector<shared_ptr<Action>> mActions;
 public:
  ParallelAction(vector<shared_ptr<Action>> actions);
  bool isFinished();
  void update();
  void done();
  void start();
};
  