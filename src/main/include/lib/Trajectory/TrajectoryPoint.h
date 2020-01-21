/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <memory>
#include <vector>
#include "lib/Trajectory/Timing/TimedState.h"
using namespace std;

class TrajectoryPoint {
 public:
  shared_ptr<TimedState> state_= make_shared<TimedState>();
  int index_=0;
  TrajectoryPoint(shared_ptr<TimedState> state, int index);
  shared_ptr<TimedState> state();
  int index();
};
