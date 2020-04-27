/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <memory>
#include "lib/Trajectory/Timing/TimedState.h"
using namespace std;

class TrajectorySamplePoint {
 protected:
  shared_ptr<TimedState> state_= make_shared<TimedState>();
  int index_floor_=0;
  int index_ceil_=0;
 public:
  TrajectorySamplePoint(shared_ptr<TimedState> state, int floor, int ceil);
  TrajectorySamplePoint(shared_ptr<TimedState> state, int point);
  shared_ptr<TimedState> state();
  int index_floor();
  int index_ceil();
};
