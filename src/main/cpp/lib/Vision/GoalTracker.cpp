/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Vision/GoalTracker.h"
#include "Constants.h"

#include <cmath>

using namespace VisionTargeting;
GoalTracker::GoalTracker() {}

TrackReport::TrackReport(GoalTrack track)
{
    field_to_target = track.getSmoothedPosition();
    latest_timestamp = track.getLatestTimestamp();
    stability = track.getStability();
    id = track.getId();
}

TrackReportComparator::TrackReportComparator(double stability_weight, double age_weight, double switching_weight, int last_track_id, double current_timestamp)
{
    this->mStabilityWeight = stability_weight;
    this->mAgeWeight = age_weight;
    this->mCurrentTimestamp = current_timestamp;
    this->mSwitchingWeight = switching_weight;
    this->mLastTrackId = last_track_id;
}

double TrackReportComparator::score(TrackReport report)
{
    double stability_score = mStabilityWeight * report.stability;
    double age_score = mAgeWeight * std::fmax(0, (Constants::kMaxGoalTrackAge - (mCurrentTimestamp - report.latest_timestamp)) / Constants::kMaxGoalTrackAge);
    double switching_score = (report.id == mLastTrackId ? mSwitchingWeight : 0.0);
    return stability_score + age_score + switching_score;
}

int TrackReportComparator::compare(TrackReport o1, TrackReport o2)
{
    double diff = score(o1) - score(o2);
    // Greater than 0 if o1 is better than o2;
    if (diff < 0)
    {
        return 1;
    } else if (diff > 0)
    {
        return -1;
    } else 
    {
        return 0;
    }
}

void GoalTracker::update(double timestamp, std::vector<std::shared_ptr<Pose2D>> field_to_goals)
{
    if (field_to_goals.empty())
    {
        for (auto track : mCurrentTracks)
        {
            track.emptyUpdate();
        }
    }
    //Try to update existing tracks
    for (auto target : field_to_goals)
    {
        bool hasUpdatedTrack = false;
        for (auto track : mCurrentTracks)
        {
            if (!hasUpdatedTrack)
            {
                if (track.tryUpdate(timestamp, target))
                {
                    hasUpdatedTrack = true;
                }
            } else
            {
                track.emptyUpdate();
            }
            
        }
        if (!hasUpdatedTrack)
        {
            mCurrentTracks.push_back(GoalTrack::makeNewTrack(timestamp, target, mNextId));
            ++mNextId;
        }
        
    }

    auto it = mCurrentTracks.begin();
    for (auto track : mCurrentTracks)
    {
        if (!track.isAlive())
        {
            mCurrentTracks.erase(it);
        } else
        {
            it++;    
        }
    }
}

std::vector<VisionTargeting::TrackReport> GoalTracker::getTracks()
{
    std::vector<TrackReport> reports;
    for (auto track : mCurrentTracks)
    {
        reports.push_back(TrackReport{track});
    }
    return reports;
}