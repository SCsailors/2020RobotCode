/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/WinchSystem.h"
using namespace Subsystems;
std::shared_ptr<WinchSystem> WinchSystem::mInstance;
WinchSystem::WinchSystem(std::shared_ptr<TalonConstants> constants) : TalonSRXSubsystem(constants) {}

std::shared_ptr<WinchSystem> WinchSystem::getInstance()
{
    if (!mInstance)
    {
        std::shared_ptr<Subsystems::TalonConstants> constants = std::make_shared<Subsystems::TalonConstants>();
        constants->kName = "Winch System";
        std::shared_ptr<Subsystems::SlaveConstants> slave = std::make_shared<Subsystems::SlaveConstants>(27, false, true);
        constants->kSlaveIDs.push_back(slave);
        constants->id = 26;
        constants->kContinuousCurrentLimit = 20;
        constants->kPeakCurrentLimit = 40;
        constants->kIsTalonSRX = true;

        mInstance = std::make_shared<WinchSystem>(constants);
    }
    return mInstance;
}

void WinchSystem::OnStart(double timestamp)
{

}

void WinchSystem::OnLoop(double timestamp)
{
    updateCurrentState();
    mDesiredState = mStateMachine.updateClimber(timestamp, mWantedAction, mCurrentState);
    mWantedAction = mStateMachine.getWantedAction(); // in case statemachine transitions
    followSetpoint();

}

void WinchSystem::OnStop(double timestamp)
{
    stop();

    mWantedAction = StateMachines::ClimberStateMachine::WantedAction::WANTED_POST_CLIMB;
}

void WinchSystem::setWantedAction(StateMachines::ClimberStateMachine::WantedAction wantedAction)
{
    mWantedAction = wantedAction;
}

void WinchSystem::updateCurrentState()
{
    mCurrentState.winch = getPosition();
}

void WinchSystem::followSetpoint()
{
    setOpenLoop(mDesiredState.winch); //Tune Feedforward
    if (mDesiredState.extend)
    {
        extendClimber();
    } else
    {
        retractClimber();
    }
    
}

void WinchSystem::extendClimber()
{
    if (!mClimberExtended)
    {
        mClimber.Set(true);
        mClimberExtended = true;
    }
}

void WinchSystem::retractClimber()
{
    if (mClimberExtended)
    {
        mClimber.Set(false);
        mClimberExtended = false;
    }
}

bool WinchSystem::isClimberExtended()
{
    return mClimberExtended;
}