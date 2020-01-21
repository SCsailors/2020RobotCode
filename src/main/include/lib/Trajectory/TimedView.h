/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Util/Util.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Trajectory/TrajectorySamplePoint.h"
#include "lib/Trajectory/TrajectoryPoint.h"
#include <memory>
#include <vector>
using namespace std;

class TimedView {
 protected:
  vector<shared_ptr<TimedState>> trajectory_;
  double start_t_=0.0;
  double end_t_=0.0;
 public:
  TimedView(vector<shared_ptr<TimedState>> trajectory);
  double first_interpolant();
  double last_interpolant();

  vector<shared_ptr<TimedState>> trajectory();
  shared_ptr<TrajectorySamplePoint> sample(double t);

};
