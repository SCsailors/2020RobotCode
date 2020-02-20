/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/LED.h"

#include <frc/smartdashboard/SmartDashboard.h>

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
        case DISPLAYING_START:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Start");
            setStartLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_FAULT:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Fault");
            setFaultLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_HANG:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Hang");
            setHangLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_INTAKING:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Intaking");
            setIntakingLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_SHOOTING:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Shooting");
            setShootingLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_ZEROING:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Zeroing");
            setZeroingLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_BALLS:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Balls");
            setBallsLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_WHEEL:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying Wheel");
            setWheelLEDCommand(mDesiredLEDState, timeInState);
            break;
        case DISPLAYING_END:
            frc::SmartDashboard::PutString("Subystems/LED/State", "Displaying End");
            setEndLEDCommand(mDesiredLEDState, timeInState);
            break;
        default:
            std::cout<<"Fell through on LED commands: "<<mSystemState<<std::endl;
            break;
    }
    

    blinkinLEDIntake.Set(mDesiredLEDState.PWM);
    blinkinLEDShooter.Set(mDesiredLEDState.PWM);
}

void LED::OnStop(double timestamp){}

void LED::setStartLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    mStartEndLEDState->getCurrentLEDState(desiredLEDState, timeInState);
}

void LED::setFaultLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    mFaultLEDState->getCurrentLEDState(desiredLEDState, timeInState);
}

void LED::setHangLEDCommand(LEDState &desiredLEDState, double timeInState)
{ 
    mHangLEDState->getCurrentLEDState(desiredLEDState, timeInState);
}

void LED::setIntakingLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    frc::SmartDashboard::PutNumber("Intake DesiredLED PWM: before", desiredLEDState.PWM);
    mIntakeLEDState->getCurrentLEDState(desiredLEDState, timeInState);
    frc::SmartDashboard::PutNumber("Intake DesiredLED PWM: after", desiredLEDState.PWM);
}

void LED::setShootingLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    mShootingLEDState->getCurrentLEDState(desiredLEDState, timeInState);
}

void LED::setZeroingLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    if (std::isnan(mLastZeroTime))
    {
        mFaultLEDState->getCurrentLEDState(desiredLEDState, timeInState);
    } else if (frc::Timer::GetFPGATimestamp() - timeInState < 3.0)
    {
        mJustZeroedLEDState->getCurrentLEDState(desiredLEDState, timeInState);
    } else
    {
        mRobotZeroedLEDState->getCurrentLEDState(desiredLEDState, timeInState);
    }
    
}

void LED::setBallsLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    mBallsLEDState->getCurrentLEDState(desiredLEDState, timeInState);
}

void LED::setWheelLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    mWheelLEDState->getCurrentLEDState(desiredLEDState, timeInState);
}

void LED::setEndLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    mStartEndLEDState->getCurrentLEDState(desiredLEDState, timeInState);
}

void LED::setOffLEDCommand(LEDState &desiredLEDState, double timeInState)
{
    mStaticOff->getCurrentLEDState(desiredLEDState, timeInState);
}

LED::SystemState LED::getStateTransition()
{
    switch (mWantedAction){
        case DISPLAY_START:
            return SystemState::DISPLAYING_START;
        case DISPLAY_FAULT:
            return SystemState::DISPLAYING_FAULT;
        case DISPLAY_HANG:
            return SystemState::DISPLAYING_HANG;
        case DISPLAY_INTAKING:
            return SystemState::DISPLAYING_INTAKING;
        case DISPLAY_SHOOTING:
            return SystemState::DISPLAYING_SHOOTING;
        case DISPLAY_ZEROING:
            return SystemState::DISPLAYING_ZEROING;
        case DISPLAY_BALLS:
            return SystemState::DISPLAYING_BALLS;
        case DISPLAY_WHEEL:
            return SystemState::DISPLAYING_WHEEL;
        case DISPLAY_END:
            return SystemState::DISPLAYING_END;
        default:
            std::cout<<"Fell through on LED wanted action check: " << mWantedAction <<std::endl;
            return SystemState::DISPLAYING_INTAKING;
    }
}
