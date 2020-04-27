/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/TrajectoryIterator.h"

TrajectoryIterator::TrajectoryIterator(shared_ptr<TimedView> view) {
    view_=view;
    current_sample_=view_->sample(view_->first_interpolant());
    progress_=view_->first_interpolant();
}

bool TrajectoryIterator::isDone(){
    return getRemainingProgress()==0.0;
}

double TrajectoryIterator::getProgress(){
    return progress_;
}

double TrajectoryIterator::getRemainingProgress(){
    return fmax(0.0, view_->last_interpolant()-progress_);
}

shared_ptr<TrajectorySamplePoint> TrajectoryIterator::getSample(){
    return current_sample_;
}

shared_ptr<TimedState> TrajectoryIterator::getState(){
    return getSample()->state();
}

shared_ptr<TrajectorySamplePoint> TrajectoryIterator::advance(double additional_progress){
    progress_=fmax(view_->first_interpolant(), 
        fmin(view_->last_interpolant(), progress_+additional_progress));
    current_sample_=view_->sample(progress_);
    return current_sample_;
}

shared_ptr<TrajectorySamplePoint> TrajectoryIterator::preview(double additional_progress){
    double progress = fmax(view_->first_interpolant(), 
        fmin(view_->last_interpolant(), progress_+additional_progress));
        return view_->sample(progress);
}

vector<shared_ptr<TimedState>> TrajectoryIterator::Trajectory(){
    return view_->trajectory();
}