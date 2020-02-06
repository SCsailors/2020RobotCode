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
  //http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  
  static LEDState kOff;
  static LEDState kIntaking;
  static LEDState kShooting;
  static LEDState kRobotZeroed;
  static LEDState kFault;
  static LEDState kHanging;
  static LEDState kStowingIntake;
  static LEDState kWheelPosition;
  static LEDState kWheelRotation;
  static LEDState kWheelComplete;
};
