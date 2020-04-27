/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/Joystick.h>
#include <frc/Buttons/JoystickButton.h>


class Joysticks {
 public:
  Joysticks();
  frc::Joystick& GetLeft();
  frc::Joystick& GetRight();
  frc::Joystick& GetGamepad();

  frc::Joystick left{1};
    frc::Joystick right{0};
    frc::Joystick gamepad{2};
    frc::JoystickButton righttrig{&right,1};
    frc::JoystickButton a {&gamepad,2};
    frc::JoystickButton b {&gamepad,3};
    frc::JoystickButton y {&gamepad,4};
    frc::JoystickButton x {&gamepad,1};
    frc::JoystickButton lb {&gamepad,5};
    frc::JoystickButton rb {&gamepad,6};
    frc::JoystickButton lt {&gamepad,7};
    frc::JoystickButton rt {&gamepad,8};
};
