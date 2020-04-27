/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Geometry/Pose2DWithCurvature.h"
#include <memory>

class ConstrainedState {
 public:
  ConstrainedState();
  shared_ptr<Pose2DWithCurvature> state=make_shared<Pose2DWithCurvature>();
  double distance=0.0;
  double max_velocity=0.0;
  double min_acceleration=0.0;
  double max_acceleration=0.0;

  
};
