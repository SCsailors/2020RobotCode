/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "StateMachines/WheelStateMachine.h"

#include "Constants.h"
using namespace StateMachines;

WheelStateMachine::WheelStateMachine() 
{
    mLED = Subsystems::LED::getInstance();
    mCurrentStateStartTime = frc::Timer::GetFPGATimestamp();
}

WheelState WheelStateMachine::updateWheel(double timestamp, WantedAction wantedAction, WheelState currentState)
{
    SystemState newState = mSystemState;
    double timeInState = timestamp - mCurrentStateStartTime;

    switch (mSystemState)
    {
        case PRE_WHEEL:
            newState = handlePreWheelStateTransitions(timestamp, timeInState, wantedAction, currentState);
            break;
        case ROTATION:
            newState = handlePreWheelStateTransitions(timestamp, timeInState, wantedAction, currentState);
            break;
        case POSITION:
            newState = handlePreWheelStateTransitions(timestamp, timeInState, wantedAction, currentState);
            break;
        case POST_WHEEL:
            newState = handlePreWheelStateTransitions(timestamp, timeInState, wantedAction, currentState);
            break;
        default:
            std::cout << "Unexpected Wheel system state " << mSystemState << std::endl;
            newState = mSystemState;
            break;
    }

    if (newState != mSystemState)
    {
        std::cout << timestamp << ": Wheel Changed State" << mSystemState << " -> " << newState << std::endl;
        mSystemState = newState;
        mCurrentStateStartTime = frc::Timer::GetFPGATimestamp();
        timeInState = 0.0;
    }

    switch (mSystemState)
    {
        case PRE_WHEEL:
            getPreWheelDesiredState(currentState, timeInState, mDesiredState);
            break;
        case ROTATION:
            getRotationDesiredState(currentState, timeInState, mDesiredState);
            break;
        case POSITION:
            getPositionDesiredState(currentState, timeInState, mDesiredState);
            break;
        case POST_WHEEL:
            getPostWheelDesiredState(currentState, timeInState, mDesiredState);
            break;
        default:
            std::cout << "Unexpected Wheel system state: " << mSystemState << std::endl;
            break;
    }
    return mDesiredState;
}

WheelStateMachine::SystemState WheelStateMachine::defaultTransition(WantedAction wantedAction)
{
    switch (wantedAction)
    {
        case WANTED_PRE_WHEEL:
            mWantedAction = WantedAction::WANTED_PRE_WHEEL;
            return SystemState::PRE_WHEEL;
        case WANTED_ROTATION:
            mWantedAction = WantedAction::WANTED_ROTATION;
            return SystemState::ROTATION;
        case WANTED_POSITION:
            mWantedAction = WantedAction::WANTED_POSITION;
            return SystemState::POSITION;
        case WANTED_POST_WHEEL:
            mWantedAction = WantedAction::WANTED_POST_WHEEL;
            return SystemState::POST_WHEEL;
    }
    return SystemState::POST_WHEEL;
}

WheelStateMachine::SystemState WheelStateMachine::handlePreWheelStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState)
{
    return defaultTransition(wantedAction);
}

void WheelStateMachine::getPreWheelDesiredState(WheelState currentState, double timeInState, WheelState &desiredState)
{
    mDesiredState.wheelDemand = 0.0;
    mDesiredState.extendWheel = true;

    Subsystems::Superstructure::getInstance()->setStateMachineLEDMaxPriority(true);
    mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_WHEEL);

    if (timeInState < .4)
    {
        mLED->setWheelLEDState(mLEDStates.kWheelStarted);
    } else
    {
        mLED->setWheelLEDState(mLEDStates.kWheelReady);
    }
    
}

WheelStateMachine::SystemState WheelStateMachine::handleRotationStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState)
{
    if (currentState.colorCount == 8)
    {
        RotPosToPostClimb = true;
        mWantedAction = WantedAction::WANTED_POST_WHEEL;
        return SystemState::POST_WHEEL;
    }
    return defaultTransition(wantedAction);
}

void WheelStateMachine::getRotationDesiredState(WheelState currentState, double timeInState, WheelState &desiredState)
{
    mDesiredState.wheelDemand = Constants::kWheelDefault;
    mDesiredState.extendWheel = true;

    mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_WHEEL);

    mLED->setWheelLEDState(mLEDStates.kWheelRotation);  
}

WheelStateMachine::SystemState WheelStateMachine::handlePositionStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState)
{
    if (currentState.colorGoal == WheelState::Color::NONE)
    {
        std::cout << "Can't do Position, no color Goal!" << std::endl;
        mWantedAction = WantedAction::WANTED_POST_WHEEL;
        return SystemState::POST_WHEEL;
    }
    
    if (currentState.color == currentState.colorGoal)
    {
        RotPosToPostClimb = true;
        mWantedAction = WantedAction::WANTED_POST_WHEEL;
        return SystemState::POST_WHEEL;
    }

    
    return defaultTransition(wantedAction);
}

void WheelStateMachine::getPositionDesiredState(WheelState currentState, double timeInState, WheelState &desiredState)
{
    mDesiredState.wheelDemand = Constants::kWheelDefault;
    mDesiredState.extendWheel = true;

    mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_WHEEL);

    mLED->setWheelLEDState(mLEDStates.kWheelPosition);
}

WheelStateMachine::SystemState WheelStateMachine::handlePostWheelStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, WheelState currentState)
{
    return defaultTransition(wantedAction);
}

void WheelStateMachine::getPostWheelDesiredState(WheelState currentState, double timeInState, WheelState &desiredState)
{
    mDesiredState.wheelDemand = 0.0;
    mDesiredState.extendWheel = false;
    //handled outside
    //mDesiredState.colorCount = 0;
    //mDesiredState.colorGoal = WheelState::Color::NONE;

    Subsystems::Superstructure::getInstance()->setStateMachineLEDMaxPriority(true);
    mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_WHEEL);

    if (RotPosToPostClimb && timeInState < 1.0)
    {
        mLED->setWheelLEDState(mLEDStates.kWheelComplete);
    } else
    {
        mLED->setWheelLEDState(mLEDStates.kIdle);
    }
}
