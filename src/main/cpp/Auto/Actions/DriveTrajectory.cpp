/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/DriveTrajectory.h"

DriveTrajectory::DriveTrajectory(vector<shared_ptr<TimedState>> trajectory) {
    mResetPose=false;
    shared_ptr<TimedView> tmp= make_shared<TimedView>(trajectory);
    mTrajectory=make_shared<TrajectoryIterator>(tmp);
}

DriveTrajectory::DriveTrajectory(vector<shared_ptr<TimedState>> trajectory, bool resetPose){
    mResetPose=resetPose;
    shared_ptr<TimedView> tmp= make_shared<TimedView>(trajectory);
    mTrajectory=make_shared<TrajectoryIterator>(tmp);
}

void DriveTrajectory::start(){
    cout << "Starting Trajectory! (Length="+ toString(mTrajectory->getRemainingProgress())+")"<<endl;
    if(mResetPose){
        Robot::robotState.reset(frc::Timer::GetFPGATimestamp(), mTrajectory->getState()->state()->getPose());
    }
    Robot::drive->setTrajectory(mTrajectory);
}

bool DriveTrajectory::isFinished(){
    if(Robot::drive->isDoneWithTrajectory()){
        cout<<"Trajectory Finished"<<endl;
        /*
        shared_ptr<DriveSignal> signal = make_shared<DriveSignal>();
        shared_ptr<DriveSignal> feedforward = make_shared<DriveSignal>();

        Robot::drive->setVelocity(signal, feedforward);
        */
        return true;
    }
    return false;
}

template<typename T>
string DriveTrajectory::toString(T value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}