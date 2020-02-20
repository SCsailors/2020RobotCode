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

CenteringIntake::CenteringIntake(Subsystems::SparkMaxConstants constants) : Subsystems::SparkMaxSubsystem(constants) {}

std::shared_ptr<Subsystems::CenteringIntake> CenteringIntake::getInstance()
{
    if (!mInstance)
    {
        Subsystems::SparkMaxConstants constants{};
        constants.id = 22;
        constants.kName = "Centering Intake";
        constants.kCurrentStallLimit = 20;
        constants.kSecondaryCurrentLimit = 30;
        constants.inverted = true;
        constants.kTicksPerUnitDistance = 42.0 / 9.0; //Rotation after gear reduction
        mInstance = std::make_shared<Subsystems::CenteringIntake>(constants);
    }
    return mInstance;
}
