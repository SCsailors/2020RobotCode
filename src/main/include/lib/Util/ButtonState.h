/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/buttons/JoystickButton.h>
namespace Utility{

class ButtonState : public frc::JoystickButton{
  bool prev_state_pressed = false;
  bool prev_state_released = false;
  int debounce_iterations = 0;
  int max_iterations = 3;
 public:
  ButtonState(frc::GenericHID *joystick, int buttonNumber): frc::JoystickButton{joystick, buttonNumber}{}

  bool isPressed();

  bool isHeld();

  bool isReleased();

};
}