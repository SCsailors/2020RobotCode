/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/BallPathTop.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "Constants.h"

using namespace Subsystems;
std::shared_ptr<Subsystems::BallPathTop> BallPathTop::mInstance;

BallPathTop::BallPathTop(std::shared_ptr<Subsystems::TalonConstants> constants) : Subsystems::TalonSRXSubsystem(constants) {}

std::shared_ptr<Subsystems::BallPathTop> BallPathTop::getInstance()
{
    if (!mInstance)
    {
        std::shared_ptr<Subsystems::TalonConstants> constants = std::make_shared<Subsystems::TalonConstants>();
        constants->id = 21;
        constants->kName = "Ball Path Top";
        constants->inverted = false;
        constants->kTicksPerUnitDistance = 8192.0; //Ticks to rotations;
        constants->kIsTalonSRX = true;
        constants->kStatusFrame8UpdateRate = 50;
        mInstance = std::make_shared<Subsystems::BallPathTop>(constants);
    }
    return mInstance;
}

void BallPathTop::updateFirst()
{
    if (!mFirstBreakState && mFirstBreakState != mPrevFirstBreakState)
    { //ball left
        mBallCount --;
    }
    if (mBallCount < 0)
    {
        mBallCount = 0;
        frc::DriverStation::ReportError("mBallCount dropped below zero!");
    }
    mPrevFirstBreakState = mFirstBreakState;
}

void BallPathTop::updateLast()
{
    if (mLastBreakState && mLastBreakState != mPrevLastBreakState)
    { //just entered 
        mLastBreakStartTimestamp = frc::Timer::GetFPGATimestamp();
        mBallCount++;
    }

    if (!mLastBreakState && mLastBreakState != mPrevLastBreakState)
    { //Just passed
        double passTime = frc::Timer::GetFPGATimestamp()-mLastBreakStartTimestamp;
        frc::SmartDashboard::PutNumber("Ball Detection Pass Time", passTime);
    }

    bool holding = (frc::Timer::GetFPGATimestamp()-mLastBreakStartTimestamp) > Constants::kBallDetectHoldTime;
    if (mLastBreakState && holding)
    { //ball is stuck or we have 5 balls
        mHasFiveBalls = true;
    }

    if (mBallCount > 5)
    {
        mBallCount = 5;
        frc::DriverStation::ReportError("mBallCount is greater than 5 Balls!");
    }

    mPrevLastBreakState = mLastBreakState;
}

void BallPathTop::readPeriodicInputs()
{
    mFirstBreakState = !mFirstBreak.Get(); 
    
    mLastBreakState = mLastBreak.Get();
    
    //mPreBottomState = mPreBottom.Get();

    updateLast();
    updateFirst();
    mHasBalls = mBallCount > 0;
    mHasFiveBalls = mBallCount == 5;

    TalonSRXSubsystem::readPeriodicInputs();
    
}

void BallPathTop::outputTelemetry()
{
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/Ball Count: ", getBallCount());
    TalonSubsystem::outputTelemetry();
}

int BallPathTop::getBallCount()
{
    return mBallCount;
}

bool BallPathTop::hasBalls()
{
    return mHasBalls;
}

bool BallPathTop::hasFiveBalls()
{
    return mHasFiveBalls;
}

void BallPathTop::SetBallCount(int count)
{
    mBallCount = count;
    if (mBallCount < 0)
    {
        mBallCount = 0;
        frc::DriverStation::ReportError("mBallCount dropped below zero!");
    } else if (mBallCount > 5)
    {
        mBallCount = 5;
        frc::DriverStation::ReportError("mBallCount is greater than 5 Balls!");
    }

}