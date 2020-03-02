/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "StateMachines/ClimberStateMachine.h"
#include <Constants.h>
using namespace StateMachines;
ClimberStateMachine::ClimberStateMachine() 
{
    mLED = Subsystems::LED::getInstance();
    mCurrentStateStartTime = frc::Timer::GetFPGATimestamp();
}

ClimberState ClimberStateMachine::updateClimber(double timestamp, WantedAction wantedAction, ClimberState currentState)
{
    SystemState newState = mSystemState;
    double timeInState = timestamp - mCurrentStateStartTime;

    switch (mSystemState)
    {
        case PRE_CLIMB:
            newState = handlePreClimbStateTransitions(timestamp, timeInState, wantedAction, currentState);
            break;
        case CLIMBING:
            newState = handlePreClimbStateTransitions(timestamp, timeInState, wantedAction, currentState);
            break;
        case POST_CLIMB:
            newState = handlePreClimbStateTransitions(timestamp, timeInState, wantedAction, currentState);
            break;
        default:
            std::cout << "Unexpected Climber system state: " << mSystemState <<std::endl;
            newState = mSystemState;
            break;
    }

    if (newState != mSystemState)
    {
        std::cout << timestamp << ": Climber Changed State" << mSystemState << "->" << newState << std::endl;
        mSystemState = newState;
        mCurrentStateStartTime = frc::Timer::GetFPGATimestamp();
        timeInState = 0.0;
    }

    switch (mSystemState) 
    {
        case PRE_CLIMB:
            getPreClimbDesiredState(currentState, timeInState, mDesiredState);
            break;
        case CLIMBING:
            getClimbingDesiredState(currentState, timeInState, mDesiredState);
            break;
        case POST_CLIMB:
            getPostClimbDesiredState(currentState, timeInState, mDesiredState);
            break;
        default:
            std::cout << "Unexpected Climber system state: " << mSystemState << std::endl;
            break;
    }
    return mDesiredState;
}

ClimberStateMachine::SystemState ClimberStateMachine::defaultTransition(WantedAction wantedAction)
{
    switch (wantedAction)
    {
        case WantedAction::WANTED_PRE_CLIMB:
            mWantedAction = WantedAction::WANTED_PRE_CLIMB;
            return SystemState::PRE_CLIMB;
        case WantedAction::WANTED_CLIMBING:
            mWantedAction = WantedAction::WANTED_CLIMBING;
            return SystemState::CLIMBING;
        case WantedAction::WANTED_POST_CLIMB:
            mWantedAction = WantedAction::WANTED_POST_CLIMB;
            return SystemState::POST_CLIMB;
    }
    return SystemState::POST_CLIMB;
}

ClimberStateMachine::SystemState ClimberStateMachine::handlePreClimbStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, ClimberState currentState)
{
    return defaultTransition(wantedAction);
}

void ClimberStateMachine::getPreClimbDesiredState(ClimberState currentState, double timeInState, ClimberState &desiredState)
{
    mDesiredState.extend = true;
    mDesiredState.winch = 0.0;

    Subsystems::Superstructure::getInstance()->setStateMachineLEDMaxPriority(true);
    mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_HANG);

    if (currentState.isAtDesiredState(desiredState) && timeInState > 0.5)
    {
        mLED->setHangLEDState(mLEDStates.kPreHangComplete);
    } else
    {
        mLED->setHangLEDState(mLEDStates.kPreHangStarted);
    }
    
}

ClimberStateMachine::SystemState ClimberStateMachine::handleClimbingStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, ClimberState currentState)
{
    return defaultTransition(wantedAction);
}

void ClimberStateMachine::getClimbingDesiredState(ClimberState currentState, double timeInState, ClimberState &desiredState)
{
    mDesiredState.extend = false;
    mDesiredState.winch = Constants::kWinchDefault;

    Subsystems::Superstructure::getInstance()->setStateMachineLEDMaxPriority(true);
    mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_HANG);

    mLED->setHangLEDState(mLEDStates.kClimbing);
}

ClimberStateMachine::SystemState ClimberStateMachine::handlePostClimbStateTransitions(double timestamp, double timeInState, WantedAction wantedAction, ClimberState currentState)
{
    return defaultTransition(wantedAction);
}

void ClimberStateMachine::getPostClimbDesiredState(ClimberState currentState, double timeInState, ClimberState &desiredState)
{
    mDesiredState.extend = false;
    mDesiredState.winch = currentState.winch;

    Subsystems::Superstructure::getInstance()->setStateMachineLEDMaxPriority(false);

    if (ClimbingToPostClimb && timeInState < 1.0)
    {
        mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_HANG);
        mLED->setHangLEDState(mLEDStates.kHangComplete);
    } else
    {
        ClimbingToPostClimb = false;
        mLED->setHangLEDState(mLEDStates.kIdle);
    }
    
}
