/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "States/TimedLEDState.h"
#include <cmath>

BlinkingLEDState::BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration)
{
    mStateOne.copyFrom(stateOne);
    mStateTwo.copyFrom(stateTwo);
    mDuration = duration;
}

void BlinkingLEDState::getCurrentLEDState(LEDState &desiredState, double timestamp)
{
    if (((int)std::round(timestamp/mDuration))%2 == 0)
    {
        desiredState.copyFrom(mStateOne);
    } else
    {
        desiredState.copyFrom(mStateTwo);
    }
    
}

StaticLEDState::StaticLEDState(LEDState staticState)
{
    mStaticState.copyFrom(staticState);
}

void StaticLEDState::getCurrentLEDState(LEDState &desiredState, double timestamp)
{
    desiredState.copyFrom(mStaticState);
}


TimedLEDStates::TimedLEDStates()
{
    kFault = std::make_shared<BlinkingLEDState>(Off, Fault, kSlowBlink);   
    kJustZeroed = std::make_shared<BlinkingLEDState>(Off, Zeroed, kMedBlink);
    kIntakingOne = std::make_shared<BlinkingLEDState>(Intaking, OneBall, kFastBlink);
    kIntakingTwo = std::make_shared<BlinkingLEDState>(Intaking, TwoBalls, kFastBlink);
    kIntakingThree = std::make_shared<BlinkingLEDState>(Intaking, ThreeBalls, kFastBlink);
    kIntakingFour = std::make_shared<BlinkingLEDState>(Intaking, FourBalls, kFastBlink);
    kIntakingFive = std::make_shared<BlinkingLEDState>(Intaking, FiveBalls, kFastBlink);
    kStowingZero = std::make_shared<BlinkingLEDState>(StowingIntake, Off, kFastBlink);
    kStowingOne = std::make_shared<BlinkingLEDState>(StowingIntake, OneBall, kFastBlink);
    kStowingTwo = std::make_shared<BlinkingLEDState>(StowingIntake, TwoBalls, kFastBlink);
    kStowingThree = std::make_shared<BlinkingLEDState>(StowingIntake, ThreeBalls, kFastBlink);
    kStowingFour = std::make_shared<BlinkingLEDState>(StowingIntake, FourBalls, kFastBlink);
    kStowingFive = std::make_shared<BlinkingLEDState>(StowingIntake, FiveBalls, kFastBlink);
    kShooterReady = std::make_shared<BlinkingLEDState>(Off, PreShooting, kFastBlink);
    kShootingComplete = std::make_shared<BlinkingLEDState>(Off, Shooting, kFastBlink);
    kWheelReady = std::make_shared<BlinkingLEDState>(Off, WheelStart, kFastBlink);
    kWheelComplete = std::make_shared<BlinkingLEDState>(Off, WheelComplete, kFastBlink);
    kPreHangComplete = std::make_shared<BlinkingLEDState>(Off, ReadyToHang, kMedBlink);
    kHangComplete = std::make_shared<BlinkingLEDState>(Off, Hanging, kMedBlink);


    kStaticOff = std::make_shared<StaticLEDState>(Off);
    kRobotZeroed = std::make_shared<StaticLEDState>(Zeroed);
    kIdle = std::make_shared<StaticLEDState>(Start);
    kEnd = std::make_shared<StaticLEDState>(End);
    kIntakeOut = std::make_shared<StaticLEDState>(Intaking);
    kStowing = std::make_shared<StaticLEDState>(StowingIntake);
    kOneBall = std::make_shared<StaticLEDState>(OneBall);
    kTwoBalls = std::make_shared<StaticLEDState>(TwoBalls);
    kThreeBalls = std::make_shared<StaticLEDState>(ThreeBalls);
    kFourBalls = std::make_shared<StaticLEDState>(FourBalls);
    kFiveBalls = std::make_shared<StaticLEDState>(FiveBalls);
    kPreShooting = std::make_shared<StaticLEDState>(PreShooting);
    kShooting = std::make_shared<StaticLEDState>(Shooting);
    kWheelStarted = std::make_shared<StaticLEDState>(WheelStart);
    kWheelPosition = std::make_shared<StaticLEDState>(WheelPosition);
    kWheelRotation = std::make_shared<StaticLEDState>(WheelRotation);
    kPreHangStarted = std::make_shared<StaticLEDState>(ReadyToHang);
    kClimbing = std::make_shared<StaticLEDState>(Hanging);


     
}
