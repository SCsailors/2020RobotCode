/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <memory>
#include <vector>
#include <cmath>

#include "lib/Trajectory/Timing/ConstrainedState.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Trajectory/Timing/TimingConstraint.h"
using namespace std;
 
class TimingUtil {
  double kEpsilon=1e-6;
  int i =-1;
  
  
  vector<shared_ptr<ConstrainedState>> constraint_states;
 public:
  TimingUtil();
  vector<shared_ptr<TimedState>> timed_states;
  vector<shared_ptr<TimedState>> timeParameterizeTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2DWithCurvature>> states,
    shared_ptr<TimingConstraint> constraints,
    double start_velocity,
    double end_velocity,
    double max_velocity,
    double max_abs_acceleration);
};



