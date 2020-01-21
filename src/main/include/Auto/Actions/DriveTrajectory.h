/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Action.h"
#include "Robot.h"
#include "lib/Trajectory/TrajectoryIterator.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Trajectory/TimedView.h"

#include "frc/Timer.h"

#include <memory>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
using namespace std;

class DriveTrajectory :public Action {
  shared_ptr<TrajectoryIterator> mTrajectory;
  bool mResetPose;
 public:
  DriveTrajectory(vector<shared_ptr<TimedState>> trajectory);
  DriveTrajectory(vector<shared_ptr<TimedState>> trajectory, bool resetPose);

  void start();
  void update(){}
  void done(){}
  bool isFinished();

  template<typename T>
  string toString(T value);
};
  