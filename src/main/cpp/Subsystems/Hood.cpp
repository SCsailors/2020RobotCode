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

Hood::Hood(std::shared_ptr<Subsystems::SparkMaxConstants> constants) : Subsystems::SparkMaxSubsystem(constants) 
{

}

std::shared_ptr<Subsystems::Hood> Hood::getInstance()
{
    if (!mInstance)
    {
        std::shared_ptr<Subsystems::SparkMaxConstants> constants = std::make_shared<Subsystems::SparkMaxConstants>();
        constants->id = 23;
        constants->kName = "Hood";
        constants->kTicksPerUnitDistance = 1.0 / 360.0; //Ticks to degrees;
        constants->kMotorType = rev::CANSparkMax::MotorType::kBrushed;
        constants->kEncoderType = rev::CANEncoder::EncoderType::kQuadrature;
        constants->kCountsPerRev = 8192.0;
        //constants->kMinUnitsLimit = 0.0;
        constants->kMaxUnitsLimit = 80.0;

        constants->kEnableForwardSoftLimit = true;
        //constants->kEnableReverseSoftLimit = true;

        constants->inverted = true;
        constants->kHomePosition = 0.0;

        constants->kP.at(1) = 9.0;//.01; //20.0;
        constants->kI.at(1) = .03;//.0001; //.03;
        constants->kD.at(1) = 20.0;//.1; // 40.0;
        constants->kMaxIAccum.at(1) = 60.0;
        constants->kIZone.at(1) = 1.0/36.0;
        constants->kClosedLoopRampRate = .1;

        constants->kCurrentFreeLimit = 15.0;
        constants->kCurrentStallLimit = 30.0;
        constants->kSecondaryCurrentLimit = 40.0;

        constants->kMaxVelocity = 30.0; //rps
        constants->kMaxAcceleration = 80.0; 
        constants->kAllowableClosedLoopError = .0027; // 1 degree
        constants->kP.at(3) = 0.0;
        constants->kI.at(3) = 0.0;
        constants->kD.at(3) = 0.0;
        constants->kF.at(3) = .0001;


        mInstance = std::make_shared<Subsystems::Hood>(constants);
    }
    return mInstance;
}

double Hood::getAngle()
{
    return getPosition();
}