/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/OpenLoopDrive.h"

OpenLoopDrive::OpenLoopDrive(double left, double right, double duration, bool finishCondition) {
    double mLeft=left;
    double mRight= right;
    double mDuration=duration;
    bool mFinishedCondition=finishCondition;
}

void OpenLoopDrive::start(){
    
    Robot::drive->setOpenLoop(make_shared<DriveSignal>(mLeft, mRight));
    mStartTime=frc::Timer::GetFPGATimestamp();
}

void OpenLoopDrive::update(){}

void OpenLoopDrive::done(){
    
    Robot::drive->setOpenLoop(make_shared<DriveSignal>(0.0, 0.0));
}

bool OpenLoopDrive::isFinished(){
    return (frc::Timer::GetFPGATimestamp()-mStartTime)>mDuration || mFinishCondition;
}
