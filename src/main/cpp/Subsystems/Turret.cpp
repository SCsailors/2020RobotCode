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

Turret::Turret(std::shared_ptr<Subsystems::SparkMaxConstants> constants): Subsystems::SparkMaxSubsystem(constants) {}

std::shared_ptr<Subsystems::Turret> Turret::getInstance()
{
    if(!mInstance)
    {
        std::shared_ptr<Subsystems::SparkMaxConstants> constants = std::make_shared<Subsystems::SparkMaxConstants>();
        constants->id = 24;
        constants->kName = "Turret";
        constants->kTicksPerUnitDistance =  1.0 * 461.0 / 360.0; //Rotations of motor to degrees; Division bigger, Multiplication smaller 
        constants->kAllowableClosedLoopError = .5; //approximately .4 degrees
        constants->kP[1] = 0.04;
        constants->kI[1] = 0.0;
        constants->kD[1] = 0.27;
        constants->kF[1] = .0000;
        
        constants->kMaxVelocity = 6000.0; //Tune rpm
        constants->kMaxAcceleration = 15000.0; //Tune rpm
        constants->kP[3] = 0.0;
        constants->kI[3] = 0.0;
        constants->kD[3] = 0.0;
        constants->kF[3] = .0001;
        
         

        constants->kEnableForwardSoftLimit = true;
        constants->kMaxUnitsLimit = 165.0;

        constants->kEnableReverseSoftLimit = true;
        constants->kMinUnitsLimit = -45.0;

        constants->kClosedLoopRampRate = .5;

        //constants->kIsAltEncoder = false;
        //constants->kCountsPerRev = 4096.0;
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
    return mConstants->kMinUnitsLimit;
}

double Turret::getMaxUnits()
{
    return mConstants->kMaxUnitsLimit;
}