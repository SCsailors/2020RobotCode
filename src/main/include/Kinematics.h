/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Twist2D.h"
#include "Constants.h"

#include <memory>
#include <cmath>
using namespace std;

class Kinematics {
  double kEpsilon= 1E-9;
  shared_ptr<Constants> constants = make_shared<Constants>();
 public:
  Kinematics();
  //Forward Kinematics using only encoders: less accurate, but useful for predicting
  shared_ptr<Twist2D> forwardKinematics(double left_wheel_delta, double right_wheel_delta);
  shared_ptr<Twist2D> forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads);
  shared_ptr<Twist2D> forwardKinematics(shared_ptr<Rotation2D> prev_heading, double left_wheel_delta, double right_wheel_delta, shared_ptr<Rotation2D> current_heading);

  shared_ptr<Pose2D> integrateForwardKinematics(shared_ptr<Pose2D> current_pose, shared_ptr<Twist2D> forward_kinematics);
  
};
