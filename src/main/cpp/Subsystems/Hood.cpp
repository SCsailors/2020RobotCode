/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Hood.h"
#include "Constants.h"
using namespace Subsystems;
std::shared_ptr<Subsystems::Hood> Hood::mInstance;

Hood::Hood(Subsystems::SparkMaxConstants constants) : Subsystems::SparkMaxSubsystem(constants) 
{

}

std::shared_ptr<Subsystems::Hood> Hood::getInstance()
{
    if (!mInstance)
    {
        Subsystems::SparkMaxConstants constants{};
        constants.id = 23;
        constants.kName = "Hood";
        constants.kTicksPerUnitDistance = 8192.0 / 360.0; //Ticks to degrees;
        constants.kMotorType = rev::CANSparkMax::MotorType::kBrushed;
        constants.kEncoderType = rev::CANEncoder::EncoderType::kQuadrature;
        constants.kCountsPerRev = 8192.0;
        constants.kCurrentFreeLimit = 15.0;
        constants.kCurrentStallLimit = 30.0;
        constants.kSecondaryCurrentLimit = 40.0;
        mInstance = std::make_shared<Subsystems::Hood>(constants);
    }
    return mInstance;
}

double Hood::getAngle()
{
    return getPosition();
}