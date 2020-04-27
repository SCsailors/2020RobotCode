/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <cmath>
#include <memory>

#include "lib/Trajectory/TimedView.h"
#include "lib/Trajectory/TrajectorySamplePoint.h"

using namespace std;

class TrajectoryIterator {
 protected:
  shared_ptr<TimedView> view_;
  double progress_=0.0;
  shared_ptr<TrajectorySamplePoint> current_sample_;
 public:
  TrajectoryIterator(shared_ptr<TimedView> view);

  bool isDone();
  double getProgress();
  double getRemainingProgress();
  shared_ptr<TrajectorySamplePoint> getSample();
  shared_ptr<TimedState> getState();

  shared_ptr<TrajectorySamplePoint> advance(double additional_progress);
  shared_ptr<TrajectorySamplePoint> preview(double additional_progress);

  vector<shared_ptr<TimedState>> Trajectory();

};
