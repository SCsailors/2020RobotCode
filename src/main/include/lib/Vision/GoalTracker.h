/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <lib/Geometry/Pose2D.h>
#include <lib/Vision/GoalTrack.h>

#include <vector>


namespace VisionTargeting{
class TrackReport{
   public:
    std::shared_ptr<Pose2D> field_to_target;
    double latest_timestamp;
    double stability;
    int id;
    TrackReport(GoalTrack track);
    TrackReport(): 
      field_to_target(std::make_shared<Pose2D>()),
      latest_timestamp(0.0),
      stability(0.0),
      id(0) {}
  };

class TrackReportComparator{
    public:
      double mStabilityWeight;
      double mAgeWeight;
      double mCurrentTimestamp;
      double mSwitchingWeight;
      int mLastTrackId;
      TrackReportComparator(double stability_weight, double age_weight, double switching_weight, int last_track_id, double current_timestamp);

      double score(TrackReport report);

      int compare(TrackReport o1, TrackReport o2);
  };

class GoalTracker {

 public:
  GoalTracker();
  std::vector<GoalTrack> mCurrentTracks;
  int mNextId = 0;

  void reset(){mCurrentTracks.clear();}

  void update(double timestamp, std::vector<std::shared_ptr<Pose2D>> field_to_goals);

  bool hasTracks(){return !(mCurrentTracks.size()==0);}

  std::vector<TrackReport> getTracks();
};
}