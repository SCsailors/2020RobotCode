/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <lib/Util/TreeMap.h>
#include <lib/Util/Util.h>

#include <lib/Geometry/Pose2D.h>
#include "Constants.h"

#include <memory>
#include <cmath>
namespace VisionTargeting{
class GoalTrack {
  Util util{};
  TreeMap<double, std::shared_ptr<Pose2D>> mObservedPositions{100};
  std::shared_ptr<Pose2D> mSmoothedPosition = NULL;
  int mId;
 public:
  GoalTrack();

  static GoalTrack makeNewTrack(double timestamp, std::shared_ptr<Pose2D> first_observation, int id);

  void emptyUpdate(){mObservedPositions.pruneByTime();}
  bool tryUpdate(double timestamp, std::shared_ptr<Pose2D> new_observation);

  bool isAlive(){return mObservedPositions.pastObjects.size()>0;}
  void smooth();
  std::shared_ptr<Pose2D> getSmoothedPosition(){return mSmoothedPosition;}
  std::shared_ptr<Pose2D> getLatestPosition(){return mObservedPositions.getLatestObject();}
  double getLatestTimestamp(){return mObservedPositions.getLatestKey();}
  double getStability(){return std::fmin(1.0, mObservedPositions.pastObjects.size()/(Constants::kCameraFastFrameRate*Constants::kMaxGoalTrackAge));}
  int getId(){return mId;}
};
}