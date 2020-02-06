/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include <lib/Geometry/Pose2D.h>
using namespace std;

namespace VisionTargeting{

class AimingParameters {
  double range;
  shared_ptr<Pose2D> robot_to_goal;
  shared_ptr<Pose2D> field_to_goal;
  shared_ptr<Rotation2D> robot_to_goal_rotation;
  double last_seen_timestamp;
  double stability;
  shared_ptr<Rotation2D> field_to_vision_target_normal;
  int track_id;
 public:
  AimingParameters();
  AimingParameters(
  shared_ptr<Pose2D> robot_to_goal,
  shared_ptr<Pose2D> field_to_goal,
  shared_ptr<Rotation2D> field_to_vision_target_normal,
  double last_seen_timestamp,
  double stability,
  int track_id);

  shared_ptr<Pose2D> getRobotToGoal(){return robot_to_goal;}
  shared_ptr<Pose2D> getFieldToGoal(){return field_to_goal;}
  shared_ptr<Rotation2D> getRobotToGoalRotation(){return robot_to_goal_rotation;}
  shared_ptr<Rotation2D> getFieldToVisionTargetNormal(){return field_to_vision_target_normal;}
  double getLastSeenTimestamp(){return last_seen_timestamp;}
  double getStability(){return stability;}
  int getTrackID(){return track_id;}
  double getRange(){return range;}

};
}