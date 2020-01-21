/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Util/Util.h"
#include <memory>
using namespace std;

class TimedState {
  shared_ptr<Util> util=make_shared<Util>();
 public:
  TimedState(shared_ptr<Pose2DWithCurvature> state, double t, double velocity, double acceleration);
  TimedState();
  shared_ptr<Pose2DWithCurvature> state();
  void set_t(double t);
  double t();
  void set_velocity(double velocity);
  double velocity();
  void set_acceleration(double acceleration);
  double acceleration();
  shared_ptr<TimedState> interpolate(shared_ptr<TimedState> other, double x);

 protected:
  shared_ptr<Pose2DWithCurvature> state_=make_shared<Pose2DWithCurvature>();
  double t_=0.0;
  double velocity_=0.0;
  double acceleration_=0.0;
};
