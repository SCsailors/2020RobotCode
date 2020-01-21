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

StaticLEDState::StaticLEDState(LEDState staticState, double timestamp)
{
    mStaticState.copyFrom(staticState);
}

void StaticLEDState::getCurrentLEDState(LEDState &desiredState, double timestamp)
{
    desiredState.copyFrom(mStaticState);
}