/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/CenteringIntake.h"
#include "Constants.h"

using namespace Subsystems;
std::shared_ptr<Subsystems::CenteringIntake> CenteringIntake::mInstance;

CenteringIntake::CenteringIntake(Subsystems::TalonConstants constants) : Subsystems::TalonSRXSubsystem(constants) {}

std::shared_ptr<Subsystems::CenteringIntake> CenteringIntake::getInstance()
{
    if (!mInstance)
    {
        mInstance = std::make_shared<Subsystems::CenteringIntake>(*Constants::kCenteringIntakeConstants.get());
    }
    return mInstance;
}
