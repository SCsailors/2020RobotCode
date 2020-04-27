/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Controls/Joysticks.h"

Joysticks::Joysticks() {}

frc::Joystick& Joysticks::GetLeft(){
  return left;
}

frc::Joystick& Joysticks::GetRight(){
  return right;
}

frc::Joystick& Joysticks::GetGamepad(){
  return gamepad;
}

