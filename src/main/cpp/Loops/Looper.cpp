/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Loops/Looper.h"

Looper::Looper(vector<shared_ptr<Subsystems::Subsystem>> subsystem) {
    mSubsystems=subsystem;
}


void Looper::writeToLog(){
    for (shared_ptr<Subsystems::Subsystem> subsystem : mSubsystems){
        subsystem->writeToLog();
    }
}

void Looper::outputToSmartDashboard(){
    for (shared_ptr<Subsystems::Subsystem> subsystem : mSubsystems){
        subsystem->outputTelemetry();
    }
}

void Looper::OnStart(double timestamp){
    for (shared_ptr<Subsystems::Subsystem> subsystem : mSubsystems){
        subsystem->OnStart(timestamp);
        
    }
}
 
void Looper::OnLoop(){
    for (shared_ptr<Subsystems::Subsystem> subsystem : mSubsystems){
    
        subsystem->readPeriodicInputs();
        timestamp=frc::Timer::GetFPGATimestamp();
        
        subsystem->OnLoop(timestamp);
        
        subsystem->outputTelemetry();
        
        subsystem->writePeriodicOutputs();
    }
}

void Looper::OnStop(double timestamp){
    for (shared_ptr<Subsystems::Subsystem> subsystem : mSubsystems){
        subsystem->OnStop(timestamp);
    }
}

void Looper::resetSensors(){
    for (shared_ptr<Subsystems::Subsystem> subsystem : mSubsystems){
        subsystem->zeroSensors();
        
    }
}

void Looper::startEnabledLoops(){
if (!running_){
    cout<<"Starting Enabled loops"<<endl;
    timestamp=frc::Timer::GetFPGATimestamp();
    OnStart(timestamp);
    mEnabledLoops.StartPeriodic(UpdateRate);
    running_=true;
    }
}

void Looper::stopEnabledLoops(){
if(running_){
    cout<<"Stopping Enabled loops"<<endl;
    mEnabledLoops.Stop();
    timestamp=frc::Timer::GetFPGATimestamp();
    OnStop(timestamp);
    running_=false;
    }
}

void Looper::startDisabledLoops(){
    if (!disabled_running_){
    cout<<"Starting Disabled loops"<<endl;
    mEnabledLoops.StartPeriodic(UpdateRate);
    disabled_running_=true;
    } else {
        cout<<"Couldn't start disabled loops: loops were running"<<endl;
    }
}

void Looper::stopDisabledLoops(){
    if (disabled_running_){
    cout<<"Stopping Disabled loops"<<endl;
    mEnabledLoops.Stop();
    disabled_running_=false;
    }else{
        cout<<"Couldn't stop disabled loops: loops weren't running"<<endl;
    }
}
