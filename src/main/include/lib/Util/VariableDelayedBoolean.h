/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "lib/Util/LatchedBoolean.h"

#include "frc/Timer.h"
namespace Utility{

class VariableDelayedBoolean {
  double startOutputTimestamp = -1.0;
  double endOutputTimestamp = -1.0;
  Utility::LatchedBoolean start{}; //turning on
  Utility::LatchedBoolean end{true}; //turning off
  Utility::LatchedBoolean on{}; // for started

 public:
  VariableDelayedBoolean(){}
  bool update(bool value, double timestamp, double delay)
  {
    if (value)
    { //while value is on keep bumping endOutput back
      endOutputTimestamp = timestamp + delay + .1;
    }

    if (start.update(value))
    { //signal turned on
      startOutputTimestamp = timestamp + delay;
    } else if (end.update(value))
    { //signal turned off
      endOutputTimestamp = timestamp + delay;
    }

    if (timestamp >= startOutputTimestamp && timestamp <= endOutputTimestamp)
    {
      return true;
    } else 
    {
      return false;
    }
  }

  bool updateStarted(bool value, double timestamp, double delay)
  { // pulses when starts
    return on.update(update(value, timestamp, delay));
  }

  void reset()
  {
    startOutputTimestamp = -1.0;
    startOutputTimestamp = -1.0;
  }
};
}