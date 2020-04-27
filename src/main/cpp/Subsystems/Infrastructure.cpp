/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Infrastructure.h"
#include "Subsystems/Superstructure.h"
using namespace Subsystems;

std::shared_ptr<Infrastructure> Infrastructure::mInstance;
Infrastructure::Infrastructure() {}

std::shared_ptr<Subsystems::Infrastructure> Infrastructure::getInstance()
{
    if(!mInstance)
    {
        mInstance = std::make_shared<Infrastructure>();
    }
    return mInstance;
}

void Infrastructure::OnStart(double timestamp){}

void Infrastructure::OnLoop(double timestamp)
{
    bool shooting = (Subsystems::Superstructure::getInstance()->getGoal().state.shooter > 5.0);
    
    if (shooting || !manualControl)
    {
        frc::SmartDashboard::PutString("Compressor State: ", "Stopped");
        stopCompressor();
    } else 
    {
        frc::SmartDashboard::PutString("Compressor State: ", "Running");
        startCompressor();
    }
    
}

void Infrastructure::OnStop(double timestamp){}

bool Infrastructure::isManualControl()
{
    return manualControl;
}

void Infrastructure::setManualControl(bool manual)
{
    manualControl = manual;
    if(manualControl)
    {
        startCompressor();
    }
}

void Infrastructure::startCompressor()
{
    compressor.Start();
}

void Infrastructure::stopCompressor()
{
    compressor.Stop();
}