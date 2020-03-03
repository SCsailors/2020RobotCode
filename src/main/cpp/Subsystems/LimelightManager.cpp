/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/LimelightManager.h"

#include <RobotState.h>


using namespace Subsystems;
std::shared_ptr<LimelightManager> LimelightManager::mInstance;

LimelightManager::LimelightManager() 
{
    mAllLimelights.push_back(mTurretLimelight);
    mAllLimelights.push_back(mIntakeLimelight);
    std::cout << "Number of Limelights: " << mAllLimelights.size() << std::endl;
}

std::shared_ptr<LimelightManager> LimelightManager::getInstance()
{
    if(!mInstance)
    {
        mInstance = std::make_shared<LimelightManager>();
    }
    return mInstance;
}

void LimelightManager::OnStart(double timestamp)
{
    setAllLEDS(Limelight::LedMode::OFF);

    FRC_7054::RobotState::getInstance()->resetVision();
}

void LimelightManager::OnLoop(double timestamp)
{
    frc::SmartDashboard::PutNumber("CheckPoint/ VisionUpdate/ Timestamp: ", timestamp);
    frc::SmartDashboard::PutNumber("CheckPoint/ VisionUpdate/ Timestamp of photo (latency): ", getAverageLatency());
    //FRC_7054::RobotState::getInstance()->addVisionUpdate(timestamp - getAverageLatency(), getTargetInfos(), getLimelights());
    //getTargetInfos();
}

void LimelightManager::OnStop(double timestamp)
{
    setAllLEDS(Limelight::LedMode::OFF);
}

void LimelightManager::readPeriodicInputs()
{
    for (auto limelight : mAllLimelights)
    {
        limelight->readPeriodicInputs();
    }
}

void LimelightManager::writePeriodicOutputs()
{
    for (auto limelight : mAllLimelights)
    {
        limelight->writePeriodicOutputs();
    }
}

void LimelightManager::outputTelemetry()
{
    for (auto limelight : mAllLimelights)
    {
        limelight->outputTelemetry();
    }
}

void LimelightManager::triggerOutputs()
{
    for (auto limelight : mAllLimelights)
    {
        limelight->triggerOutputs();
    }
}

std::shared_ptr<Subsystems::Limelight> LimelightManager::getTurretLimelight()
{
    return mTurretLimelight;
}

std::shared_ptr<Subsystems::Limelight> LimelightManager::getIntakeLimelight()
{
    return mIntakeLimelight;
}

std::vector<std::shared_ptr<Subsystems::Limelight>> LimelightManager::getLimelights()
{
    return mAllLimelights;
}

std::vector<VisionTargeting::TargetInfo> LimelightManager::getTargetInfos()
{
    std::vector<VisionTargeting::TargetInfo> mTargets;
    for (auto limelight : mAllLimelights)
    {
        mTargets.push_back(limelight->getCameraXYZ());
    }

    return mTargets;

}

double LimelightManager::getAverageLatency()
{   
    double x = 0.0;
    double i = mAllLimelights.size();
    for (auto limelight : mAllLimelights)
    {
        x += limelight->getLatency();   
    }
    return x/i;
}

void LimelightManager::setAllLEDS(Limelight::LedMode mode)
{
    for (auto limelight : mAllLimelights)
    {
        limelight->setLed(mode);
    }
}

void LimelightManager::setPipeline(int pipeline)
{
    for (auto limelight : mAllLimelights)
    {
        limelight->setPipeline(pipeline);
    }
}