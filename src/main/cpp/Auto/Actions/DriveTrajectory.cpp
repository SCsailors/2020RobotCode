/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/DriveTrajectory.h"
#include "Subsystems/FalconDrive.h"

#include <RobotState.h>

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
        FRC_7054::RobotState::getInstance()->reset(frc::Timer::GetFPGATimestamp(), mTrajectory->getState()->state()->getPose());
    }
    Subsystems::FalconDrive::getInstance()->setTrajectory(mTrajectory);
}

bool DriveTrajectory::isFinished(){
    if(Subsystems::FalconDrive::getInstance()->isDoneWithTrajectory()){
        cout<<"Trajectory Finished"<<endl;
        
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