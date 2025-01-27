/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "States/SuperstructureState.h"

#include <cmath>

class SuperstructureGoal {
  SuperstructureState state{0.0, 0.0, 0.0};
 public:
  SuperstructureGoal(double turret, double shooter, double hood);
  SuperstructureGoal(SuperstructureState state);

  bool equals(SuperstructureGoal other);
  bool isAtDesiredState(SuperstructureState currentState);

};
