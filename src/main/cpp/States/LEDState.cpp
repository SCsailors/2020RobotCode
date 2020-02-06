/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "States/LEDState.h"

//mostly arbitrary colors except off.
LEDState LEDState::kOff{.99}; //black
LEDState LEDState::kIntaking{0.79}; //blue-green
LEDState LEDState::kShooting{0.83}; //aqua
LEDState LEDState::kRobotZeroed{0.57}; //hot pink
LEDState LEDState::kFault{0.59}; //red
LEDState LEDState::kHanging{0.91}; //violet
LEDState LEDState::kStowingIntake{0.69}; //yellow; flash number of balls
LEDState LEDState::kWheelPosition{0.71}; //lawn green; set this to the specific color?
LEDState LEDState::kWheelRotation{0.61}; //
LEDState LEDState::kWheelComplete{0.85}; //dark blue; flash specific color or number of rotations?

LEDState::LEDState(double pwm) 
{
    PWM = pwm;
}

void LEDState::copyFrom(LEDState ledState)
{
    PWM = ledState.PWM;
}
