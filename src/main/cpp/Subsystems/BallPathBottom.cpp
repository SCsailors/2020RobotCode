/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/BallPathBottom.h"
#include "Constants.h"

using namespace Subsystems;
std::shared_ptr<Subsystems::BallPathBottom> BallPathBottom::mInstance;

BallPathBottom::BallPathBottom(Subsystems::TalonConstants constants) : Subsystems::TalonSRXSubsystem(constants) 
{
    std::cout<<"BallPathBottom Construction Timestamp: " << 2.0 <<std::endl;
}

std::shared_ptr<Subsystems::BallPathBottom> BallPathBottom::getInstance()
{
    if (!mInstance)
    {
        Subsystems::TalonConstants constants{};
        constants.id = 20;
        constants.kName = "Ball Path Bottom";
        constants.kTicksPerUnitDistance = 8192.0; //Ticks to rotations;
        constants.kIsTalonSRX = true;
        constants.kStatusFrame8UpdateRate = 50;
        mInstance = std::make_shared<Subsystems::BallPathBottom>(constants);
    }
    return mInstance;
}