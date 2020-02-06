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
        mInstance = std::make_shared<Turret>(*Constants::kTurretConstants.get());
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