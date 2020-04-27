/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/AutoModeBase.h"

AutoModeBase::AutoModeBase() {}

void AutoModeBase::run(){
    mActive=true;
    routine();

    done();
}

void AutoModeBase::done(){
    cout<<"Auto Mode done"<< endl;
}
 
bool AutoModeBase::isActive(){
    return mActive;
}

void AutoModeBase::setActive(bool run){
    mActive=run;
}

void AutoModeBase::runAction(shared_ptr<Action> action){
    action->start();
    int i=0;
    int j=0;
    while (isActive() && !action->isFinished()){
        frc::SmartDashboard::PutBoolean("isActive()", isActive());
        frc::SmartDashboard::PutBoolean("! isFinished()", !action->isFinished());
        frc::SmartDashboard::PutBoolean("Active or !finished", isActive()||!action->isFinished());
        timestamp=frc::Timer::GetFPGATimestamp();
        if (timestamp-prev_timestamp>mUpdateRate){
            prev_timestamp+=mUpdateRate;
            action->update();
            } else{
            continue;
        }

    }

    action->done();
}

string AutoModeBase::getID(){
    return "AutoModeBase";
}


