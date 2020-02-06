/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/LED.h"

using namespace Subsystems;

std::shared_ptr<LED> LED::mLEDInstance;

std::shared_ptr<LED> LED::getInstance()
{
    if (!mLEDInstance)
    {
        mLEDInstance = std::make_shared<LED>();
    }

    return mLEDInstance;
}

void LED::OnStart(double timestamp)
{
    stateStartTime = timestamp;
}

void LED::OnLoop(double timestamp)
{
    SystemState newState = getStateTransition();

    if(mSystemState != newState)
    {
        std::cout<<timestamp<<": LED changed state: " << mSystemState<< " -> "<<newState<<std::endl;
        mSystemState = newState;
        stateStartTime = timestamp;
    }   

    double timeInState = timestamp - stateStartTime;

    switch (mSystemState) {
        case DISPLAYING_FAULT:
            break;
        case DISPLAYING_INTAKING:
            setIntakeLEDCommand(timeInState);
            break;
        case DISPLAYING_SHOOTING:
            break;
        case DISPLAYING_ZEROING:
            break;
        case DISPLAYING_HANG:
            break;
        default:
            std::cout<<"Fell through on LED commands: "<<mSystemState<<std::endl;
            break;
    }
    blinkinLED.Set(mDesiredLEDState.PWM);
}

void LED::OnStop(double timestamp){}

void LED::setIntakeLEDCommand(double timeInState)
{
    mIntakeLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
}

LED::SystemState LED::getStateTransition()
{
    switch (mWantedAction){
        case DISPLAY_ZEROING:
            return SystemState::DISPLAYING_ZEROING;
        case DISPLAY_HANG:
            return SystemState::DISPLAYING_HANG;
        case DISPLAY_INTAKING:
            return SystemState::DISPLAYING_INTAKING;
        case DISPLAY_FAULT:
            return SystemState::DISPLAYING_FAULT;
        case DISPLAY_SHOOTING:
            return SystemState::DISPLAYING_SHOOTING;
        default:
            std::cout<<"Fell through on LED wanted action check: " << mWantedAction <<std::endl;
            return SystemState::DISPLAYING_INTAKING;
    }
}