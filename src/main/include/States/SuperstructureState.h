/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "lib/Util/Util.h"

#include <vector>
#include <string>

class SuperstructureState {
  Util util;
 public:
  double turret; //degrees
  double shooter; //RPS
  double hood; //degrees
  double ballPathTop; //RPS
  double ballPathBottom; //RPS 
  double centeringIntake; //RPS
  SuperstructureState(double turret, double shooter, double hood, double ballPathTop, double ballPathBottom, double centeringIntake);
  SuperstructureState();
  void setFrom(SuperstructureState source);
  std::string toString();

  std::vector<double> asVector();
};
