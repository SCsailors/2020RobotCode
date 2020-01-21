/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "States/SuperstructureGoal.h"
#include "States/SuperstructureConstants.h"

SuperstructureGoal::SuperstructureGoal(double turret, double shooter, double hood) 
{
    state.turret = turret;
    state.shooter = shooter;
    state.hood = hood;
}

SuperstructureGoal::SuperstructureGoal(SuperstructureState state)
{
    this->state = state;
}

bool SuperstructureGoal::equals(SuperstructureGoal other)
{
    return this->state.turret == other.state.turret &&
            this->state.shooter == other.state.shooter &&
            this->state.hood == other.state.hood;
}

bool SuperstructureGoal::isAtDesiredState(SuperstructureState currentState)
{
    std::vector<double> error{
        currentState.turret - state.turret,
        currentState.shooter - state.shooter,
        currentState.hood - state.hood
        };
    
    for (int i = 0; i < error.size(); i++)
    {
        if ( std::fabs(error.at(i)) > SuperstructureConstants::kPadding.at(i))
        {
            return false;
        }

        return true;
    }
}