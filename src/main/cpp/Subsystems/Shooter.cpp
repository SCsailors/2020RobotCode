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
Shooter::Shooter(Subsystems::TalonConstants constants) : Subsystems::TalonFXSubsystem(constants) {}

std::shared_ptr<Subsystems::Shooter> Shooter::getInstance()
{
    if (!mInstance)
    {
        mInstance = std::make_shared<Subsystems::Shooter>(*Constants::kShooterConstants.get());
    }
    return mInstance;
}
