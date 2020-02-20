/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Turret.h"
#include "Constants.h"
using namespace Subsystems;
std::shared_ptr<Subsystems::Turret> Turret::mInstance;

Turret::Turret(Subsystems::SparkMaxConstants constants): Subsystems::SparkMaxSubsystem(constants) {}

std::shared_ptr<Subsystems::Turret> Turret::getInstance()
{
    if(!mInstance)
    {
        Subsystems::SparkMaxConstants constants{};
        constants.id = 24;
        constants.kName = "Turret";
        constants.kTicksPerUnitDistance = 4096.0; //Ticks to degrees;
        constants.kAllowableClosedLoopError = 5.0; //ticks
        constants.kMaxVelocity = 1700.0; //Tune :ticks
        constants.kMaxAcceleration = 3400.0; //Tune :ticks

        constants.kEnableForwardSoftLimit = true;
        constants.kForwardSoftLimit = 270.0;

        constants.kEnableReverseSoftLimit = true;
        constants.kReverseSoftLimit = -270.0;

        constants.kIsAltEncoder = true;
        constants.kCountsPerRev = 4096.0;
        mInstance = std::make_shared<Turret>(constants);
    }
    return mInstance;
}

double Turret::getAngle()
{
    return getPosition();
}

bool Turret::atHomingLocation()
{
    return mBannerInput.GetAverageVoltage() > 4.0;
}

void Turret::handleMasterReset(bool reset)
{
    if (mJustReset.update(reset) && kUseManualHomingRoutine)
    {
        std::cout <<"Turret going into home mode" << std::endl;
    }
}

bool Turret::isHoming()
{
    return mHoming;
}

double Turret::getMinUnits()
{
    return mConstants.kMinUnitsLimit;
}

double Turret::getMaxUnits()
{
    return mConstants.kMaxUnitsLimit;
}