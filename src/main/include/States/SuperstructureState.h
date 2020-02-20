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
  Util util{};
 public:
  double turret = 0.0; //degrees
  double shooter = 0.0; //RPS
  double hood = 0.0; //degrees
  double ballPathTop = 0.0; //RPS
  double ballPathBottom = 0.0; //RPS 
  double centeringIntake = 0.0; //RPS
  double numBalls = 0.0;
  bool extendIntake = false;
  bool extendWheelieBar = false;
  SuperstructureState(double turret, double shooter, double hood, double ballPathTop, double ballPathBottom, double centeringIntake, int numBalls, bool extendIntake, bool extendWheelieBar);
  SuperstructureState();
  void setFrom(SuperstructureState source);
  std::string toString();
  bool hasBalls();
  bool hasFiveBalls();

  bool intakeExtended(){return extendIntake;}
  bool wheelieBarExtended(){return extendWheelieBar;}
  void reset();
  

  std::vector<double> asVector();
};
