/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/TrajectorySamplePoint.h"

TrajectorySamplePoint::TrajectorySamplePoint(shared_ptr<TimedState> state, int floor, int ceil) {
    state_=state;
    index_floor_=floor;
    index_ceil_=ceil;
}

TrajectorySamplePoint::TrajectorySamplePoint(shared_ptr<TimedState> state, int point){
    state_=state;
    index_floor_=point;
    index_ceil_=point;
}

shared_ptr<TimedState> TrajectorySamplePoint::state(){
    return state_;
}

int TrajectorySamplePoint::index_floor(){
    return index_floor_;
}

int TrajectorySamplePoint::index_ceil(){
    return index_ceil_;
}