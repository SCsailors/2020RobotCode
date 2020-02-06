/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
namespace Utility{
class LatchedBoolean {
  bool mLast = false;
  bool invert = false;
 public:
 bool update(bool newState);
  LatchedBoolean(bool invert): invert(invert){} //false: rising edge (turning on), true: falling edge (turning off)
  LatchedBoolean(){};
};
}