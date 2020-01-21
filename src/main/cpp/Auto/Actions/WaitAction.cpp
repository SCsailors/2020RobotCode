/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/WaitAction.h"

WaitAction::WaitAction(double time_to_wait) {
    mTimeToWait=time_to_wait;
}

bool WaitAction::isFinished(){
    return frc::Timer::GetFPGATimestamp()-mStartTime>=mTimeToWait;
}

void WaitAction::start(){
    cout<<"Starting Wait Action"<<endl;
    mStartTime=frc::Timer::GetFPGATimestamp();
}

