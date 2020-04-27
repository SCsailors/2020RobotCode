/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Modes/PIDTuningMode.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "Robot.h"

PIDTuningMode::PIDTuningMode() {}

void PIDTuningMode::routine(){
    //runAction(make_shared<PIDTuner>(1.0, 4.0));
    
    runAction(make_shared<SeriesAction>(createProfile(setpoints)));
    
}

vector<shared_ptr<Action>> PIDTuningMode::createProfile(vector<double> percentOutput){
    frc::SmartDashboard::PutNumber("PIDTuningMode input vector size", percentOutput.size());
    vector<shared_ptr<Action>> mActions;
    for (auto output: percentOutput){
        shared_ptr<PIDTuner> tmp=  make_shared<PIDTuner>(output, 2.0);
        mActions.push_back(tmp);
        
        frc::SmartDashboard::PutNumber("PIDTuningMode counter", mActions.size());
    }
    return mActions;
}

string PIDTuningMode::getID(){
    return "PIDTuningMode";
}