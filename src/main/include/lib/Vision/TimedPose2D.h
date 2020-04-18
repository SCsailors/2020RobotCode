/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <lib/Geometry/Pose2D.h>

#include <memory>

class TimedPose2D {
  
 public:
  std::shared_ptr<Pose2D> pose;
  double timestamp;
  TimedPose2D(double timestamp, std::shared_ptr<Pose2D> pose): pose(pose), timestamp(timestamp) {}
  
};
