/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/InterpolatingTreeMap.h"

InterpolatingTreeMap::InterpolatingTreeMap(int max_size) {
    MaximumSize=max_size;
}



InterpolatingTreeMap::InterpolatingStates::InterpolatingStates(double timestamp, shared_ptr<Pose2D> state){
    timestamp_=timestamp;
    state_=state;
}

void InterpolatingTreeMap::put(double timestamp, shared_ptr<Pose2D> state){
    shared_ptr<InterpolatingTreeMap::InterpolatingStates> states_=make_shared<InterpolatingTreeMap::InterpolatingStates>(timestamp, state);
    
    if (MaximumSize>0 && MaximumSize<=pastStates.size()){
        pastStates.erase(pastStates.begin());
    }
    pastStates.push_back(states_);
}

shared_ptr<Pose2D> InterpolatingTreeMap::getInterpolated(double timestamp){
    double timestamped= pastStates.at(0)->timestamp_;
    if (timestamp<= timestamped){
        return pastStates.at(0)->state_;
    } 
    timestamped= pastStates.at(pastStates.size()-1)->timestamp_;
    if (timestamp>= timestamped){
        return pastStates.at(pastStates.size()-1)->state_;
    } else {
        for (int i=0; i<pastStates.size()-1; i++){
            timestamped= pastStates.at(i)->timestamp_;
            if(util->epsilonEquals(timestamp, timestamped)){
                return pastStates.at(i)->state_;
            } else if (timestamped> timestamp){
                shared_ptr<InterpolatingTreeMap::InterpolatingStates> tmp= make_shared<InterpolatingTreeMap::InterpolatingStates>(timestamp, pastStates.at(i-1)->state_->interpolate(pastStates.at(i)->state_, 1.0- (pastStates.at(i)->timestamp_-timestamp)/(pastStates.at(i)->timestamp_-pastStates.at(i-1)->timestamp_)));
                return tmp->state_;
            }
        }
    }
}