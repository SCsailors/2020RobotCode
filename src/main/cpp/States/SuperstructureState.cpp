/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "States/SuperstructureState.h"

SuperstructureState::SuperstructureState(double turret, double shooter, double hood) 
{
    this->turret = turret;
    this->shooter = shooter;
    this->hood = hood;
}

SuperstructureState::SuperstructureState()
{
    turret = 0.0;
    shooter = 0.0;
    hood = 0.0;
}

void SuperstructureState::setFrom(SuperstructureState source)
{
    turret = source.turret;
    shooter = source.shooter;
    hood = source.hood;
}

std::string SuperstructureState::toString()
{
    return util.toString(turret) + ", " + util.toString(shooter) + ", " + util.toString(hood) + ", ";
}

std::vector<double> SuperstructureState::asVector()
{
    std::vector<double> tmp{turret, shooter, hood};
    return tmp;
}