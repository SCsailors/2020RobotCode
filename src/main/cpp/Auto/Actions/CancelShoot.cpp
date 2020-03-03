/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/CancelShoot.h"

#include "Subsystems/Superstructure.h"

CancelShoot::CancelShoot() {}

void CancelShoot::start()
{
    Subsystems::Superstructure::getInstance()->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_IDLE);
}

void CancelShoot::update(){}

void CancelShoot::done(){}

bool CancelShoot::isFinished()
{
    return true;
}
