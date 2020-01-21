/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class LEDState {
 public:
  LEDState(double pwm);

  void copyFrom(LEDState ledState);
  double PWM;

  //add specific pwm colors and actions here as static LEDState.
};
