/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Vision/AimingParameters.h"
using namespace VisionTargeting;
AimingParameters::AimingParameters(): 
        range(0.0),
        robot_to_goal(make_shared<Pose2D>()),
        field_to_goal(make_shared<Pose2D>()),
        robot_to_goal_rotation(make_shared<Rotation2D>()),
        last_seen_timestamp(0.0),
        stability(0.0),
        field_to_vision_target_normal(make_shared<Rotation2D>())
        {}

AimingParameters::AimingParameters(shared_ptr<Pose2D> robot_to_goal,
  shared_ptr<Pose2D> field_to_goal,
  shared_ptr<Rotation2D> field_to_vision_target_normal,
  double last_seen_timestamp,
  double stability,
  int track_id) : 
        range(robot_to_goal->getTranslation()->norm()),
        robot_to_goal(robot_to_goal),
        field_to_goal(field_to_goal),
        robot_to_goal_rotation(robot_to_goal->getRotation()),
        last_seen_timestamp(last_seen_timestamp),
        stability(stability),
        field_to_vision_target_normal(field_to_vision_target_normal),
        track_id(track_id)
        {}
