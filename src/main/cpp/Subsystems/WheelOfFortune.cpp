/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/WheelOfFortune.h"
using namespace Subsystems;
std::shared_ptr<WheelOfFortune> WheelOfFortune::mInstance;

WheelOfFortune::WheelOfFortune(std::shared_ptr<Subsystems::TalonConstants> constants) : TalonSRXSubsystem(constants) {}

std::shared_ptr<WheelOfFortune> WheelOfFortune::getInstance()
{
    if (!mInstance)
    {
        std::shared_ptr<TalonConstants> constants = std::make_shared<TalonConstants>();
        constants->kName = "Wheel of Fortune";
        constants->kIsTalonSRX = true;
        constants->id = 25;
        

        mInstance = std::make_shared<WheelOfFortune>(constants);
    }
    return mInstance;
}

void WheelOfFortune::OnStart(double timestamp)
{

}

void WheelOfFortune::OnLoop(double timestamp)
{
    updateCurrentState();
    mDesiredState = mStateMachine.updateWheel(timestamp, mWantedAction, mCurrentState);
    mWantedAction = mStateMachine.getWantedAction();
    followSetpoint();
}

void WheelOfFortune::OnStop(double timestamp)
{
    stop();

    mWantedAction = StateMachines::WheelStateMachine::WANTED_POST_WHEEL;
}

void WheelOfFortune::setWantedAction(StateMachines::WheelStateMachine::WantedAction wantedAction)
{
    mWantedAction = wantedAction;
}

void WheelOfFortune::updateCurrentState()
{
    mCurrentState.wheelDemand = getPosition();
    //add color, wanted color, count, wanted  count, etc.
}

void WheelOfFortune::followSetpoint()
{
    setOpenLoop(mDesiredState.wheelDemand);
    if (mDesiredState.extendWheel)
    {
        extendWheel();
    } else
    {
        retractWheel();
    }
}

void WheelOfFortune::extendWheel()
{
    if (!mWheelExtended)
    {
        mExtensionSolenoid.Set(true);
        mWheelExtended = true;
    }
}

void WheelOfFortune::retractWheel()
{
    if (mWheelExtended)
    {
        mExtensionSolenoid.Set(false);
        mWheelExtended = false;
    }
}

bool WheelOfFortune::isWheelExtended()
{
    return mWheelExtended;
}

void WheelOfFortune::setColorGoal(StateMachines::WheelState::Color wantedColor)
{
    mDesiredColor = wantedColor;
}

StateMachines::WheelState::Color WheelOfFortune::getColorGoal()
{
    return mDesiredColor;
}

