/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Vision/GoalTrack.h"
using namespace VisionTargeting;
GoalTrack::GoalTrack() {}

std::shared_ptr<GoalTrack> GoalTrack::makeNewTrack(double timestamp, std::shared_ptr<Pose2D> first_observation, int id)
{
    std::shared_ptr<VisionTargeting::GoalTrack> rv = std::make_shared<VisionTargeting::GoalTrack>();
    rv->mObservedPositions.put(timestamp, first_observation);
    rv->mSmoothedPosition = first_observation;
    rv->mId = id;
    return rv;
}

bool GoalTrack::tryUpdate(double timestamp, std::shared_ptr<Pose2D> new_observation)
{
    if( !isAlive())
    {
        return false;
    }
    double distance = mSmoothedPosition->inverse()->transformBy(new_observation)->getTranslation()->norm();
    if(distance < Constants::kMaxTrackerDistance && 
            util.epsilonEquals(mSmoothedPosition->getRotation()->getDegrees(), new_observation->getRotation()->getDegrees(), Constants::kMaxThetaError) && 
            util.epsilonEquals(mSmoothedPosition->getTranslation()->y(), new_observation->getTranslation()->y(), Constants::kMaxXYError) && 
            util.epsilonEquals(mSmoothedPosition->getTranslation()->x(), new_observation->getTranslation()->x(), Constants::kMaxXYError)
            ) // TODO: tune X, Y, Theta epsilons.
    {
        //std::cout << "Putting Pose" << std::endl;
        mObservedPositions.put(timestamp, new_observation);
        //mObservedPositions.pruneByTime();
        
        return true;
    } else
    {
        //emptyUpdate();
        return false;
    }
    
}

void GoalTrack::smooth()
{
    if(isAlive())
    {
        double x = 0.0;
        double y = 0.0;
        double s = 0.0;
        double c = 0.0;

        double t_now = frc::Timer::GetFPGATimestamp();
        int num_samples = 0;
        for (auto pose: mObservedPositions.pastObjects)
        {
            if (t_now-pose.mKey > Constants::kMaxGoalTrackSmoothingTime)
            {
                continue;
            }
            ++num_samples;
            x += pose.mObject->getTranslation()->x();
            y += pose.mObject->getTranslation()->y();
            c += pose.mObject->getRotation()->cos();
            s += pose.mObject->getRotation()->sin();
        }
        x /= num_samples;
        y /= num_samples;
        c /= num_samples;
        s /= num_samples;

        if (num_samples == 0)
        {
            //All samples are older than trackSmoothingTime
            mSmoothedPosition = mObservedPositions.getLatestObject();
        } else
        {
            mSmoothedPosition = std::make_shared<Pose2D>(std::make_shared<Translation2D>(x, y), std::make_shared<Rotation2D>(c,s,true));
        }
        
    }
}

