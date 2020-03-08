/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/OpenLoopDrive.h"

#include "Subsystems/FalconDrive.h"

OpenLoopDrive::OpenLoopDrive(double left, double right, double duration, bool finishCondition) {
    mLeft = left;
    mRight = right;
    mDuration = duration;
    mFinishCondition = finishCondition;

}

void OpenLoopDrive::start(){
    
    Subsystems::FalconDrive::getInstance()->setOpenLoop(make_shared<DriveSignal>(mLeft, mRight));
    //mStartTime = frc::Timer::GetFPGATimestamp();
    mTimer.Reset();
    mTimer.Start();
    frc::SmartDashboard::PutString("Action/OpenLoopDrive/Started", "Started");
    frc::SmartDashboard::PutNumber("Action/OpenLoopDrive/Started timestamp", frc::Timer::GetFPGATimestamp());
}

void OpenLoopDrive::update()
{
    frc::SmartDashboard::PutString("Action/OpenLoopDrive/Updated", "Updated");
    frc::SmartDashboard::PutNumber("Action/OpenLoopDrive/Updated timestamp", frc::Timer::GetFPGATimestamp());
}

void OpenLoopDrive::done(){    
    Subsystems::FalconDrive::getInstance()->setOpenLoop(make_shared<DriveSignal>(0.0, 0.0));
    frc::SmartDashboard::PutString("Action/OpenLoopDrive/Done", "Done");
    frc::SmartDashboard::PutNumber("Action/OpenLoopDrive/Done timestamp", frc::Timer::GetFPGATimestamp());
}

bool OpenLoopDrive::isFinished(){
    return mTimer.Get() > mDuration || mFinishCondition;
}
