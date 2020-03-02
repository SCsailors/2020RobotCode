/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Shooter.h"
#include "Constants.h"
using namespace Subsystems;
std::shared_ptr<Subsystems::Shooter> Shooter::mInstance;
Shooter::Shooter(std::shared_ptr<Subsystems::TalonConstants> constants) : Subsystems::TalonFXSubsystem(constants) 
{
    mMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, Constants::kVelocityPIDSlot);
    
}

std::shared_ptr<Subsystems::Shooter> Shooter::getInstance()
{
    if (!mInstance)
    {
        std::shared_ptr<Subsystems::TalonConstants> constants = std::make_shared<Subsystems::TalonConstants>();
        constants->id = 30;
        std::shared_ptr<Subsystems::SlaveConstants> slave1 = std::make_shared<Subsystems::SlaveConstants>(31, false, false);
        constants->kSlaveIDs.push_back(slave1);
        constants->kName = "Shooter";
        constants->kTicksPerUnitDistance = 2048.0 * 3.0 / 4.0; //Ticks to rotations: add gearing for internal sensor, unless we add a canifier and a throughbore as a remote sensor
        constants->kIsTalonSRX = false;

        constants->kPeakCurrentLimit = 60; //for TalonFX: current limit
        constants->kContinuousCurrentLimit = 40; //trigger threshold
        constants->kPeakCurrentLimit = 200; //threshold time
        constants->kStatusFrame8UpdateRate = 50;
        constants->kNeutralMode = NeutralMode::Coast;
        constants->kClosedLoopRampRate = 1.5;

        constants->kP.at(2) = .22;
        constants->kF.at(2) = .1097;
        
        mInstance = std::make_shared<Subsystems::Shooter>(constants);
    }
    return mInstance;
}
