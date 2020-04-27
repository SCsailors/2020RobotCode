/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/TrajectoryPoint.h"

#include "lib/Trajectory/Timing/TimedState.h"

TrajectoryPoint::TrajectoryPoint(shared_ptr<TimedState> state, int index) {
    state_=state;
    index_=index;
}

shared_ptr<TimedState> TrajectoryPoint::state(){
    return state_;
}

int TrajectoryPoint::index(){
    return index_;
}