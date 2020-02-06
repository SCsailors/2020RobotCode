/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "States/SuperstructureState.h"

SuperstructureState::SuperstructureState(double turret, double shooter, double hood, double ballPathBottom, double ballPathTop, double centeringIntake) 
{
    this->turret = turret;
    this->shooter = shooter;
    this->hood = hood;
    this->ballPathTop = ballPathTop;
    this->ballPathBottom = ballPathBottom;
    this->centeringIntake = centeringIntake;
}

SuperstructureState::SuperstructureState()
{
    turret = 0.0;
    shooter = 0.0;
    hood = 0.0;
    ballPathTop = 0.0;
    ballPathBottom = 0.0;
    centeringIntake = 0.0;
}

void SuperstructureState::setFrom(SuperstructureState source)
{
    turret = source.turret;
    shooter = source.shooter;
    hood = source.hood;
    ballPathTop = source.ballPathTop;
    ballPathBottom = source.ballPathBottom;
    centeringIntake = source.centeringIntake;
}

std::string SuperstructureState::toString()
{
    return util.toString(turret) + ", " + util.toString(shooter) + ", " + util.toString(hood) + ", " + util.toString(ballPathTop) + ", " + util.toString(ballPathBottom) + ", " + util.toString(centeringIntake) + ", " ;
}

std::vector<double> SuperstructureState::asVector()
{
    std::vector<double> tmp{turret, shooter, hood, ballPathTop, ballPathBottom, centeringIntake};
    return tmp;
}