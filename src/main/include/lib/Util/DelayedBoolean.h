/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
namespace Utility{
class DelayedBoolean {
  bool mLastValue = false;
  double mTransitionTimestamp;
  double mDelay;
 public:
  DelayedBoolean()
  {
    mTransitionTimestamp = 0.0;
    mDelay = 0.0;
  }
  DelayedBoolean(double timestamp, double delay)
  {
    mTransitionTimestamp = timestamp;
    mDelay = delay;
  }

  bool update(double timestamp, bool value)
  {
    bool result = false;

    if (value && !mLastValue)
    {
      mTransitionTimestamp = timestamp;
    }

    //still true and transitioned
    if (value && (timestamp - mTransitionTimestamp > mDelay))
    {
      result = true;
    }

    mLastValue = value;
    return result;
  }
  void reset(double timestamp, double delay)
  {
    mTransitionTimestamp = timestamp;
    mDelay = delay;
  }
};
}