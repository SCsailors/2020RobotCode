/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <lib/Util/TimeDelayedBoolean.h>
#include <lib/Util/LatchedBoolean.h>

namespace Utility{

class MultiTrigger {
  double mTimeout;
  bool lastPressed = false;
  LatchedBoolean mWasTappedRise{false};
  LatchedBoolean mWasTappedFall{true}; //set to falling edge
  LatchedBoolean mWasHeld{};
  bool lastTapped = false;
  TimeDelayedBoolean mIsHeld{};
  double mTapStartTime = 0.0;

 public:
  MultiTrigger(double timeout)
  {
    mTimeout = timeout;
  }

  void update(bool pressed)
  {
    lastPressed = pressed;
    
    bool fall = mWasTappedFall.update(pressed);
    bool rise = mWasTappedRise.update(pressed);

    if (rise)
    {
      mTapStartTime = frc::Timer::GetFPGATimestamp();
    }

    lastTapped = fall && (frc::Timer::GetFPGATimestamp() - mTapStartTime) < mTimeout;

    mIsHeld.update(pressed, mTimeout);
  }

  bool wasTapped()
  {
    return lastTapped;
  }

  bool isPressed()
  {
    return lastPressed;
  }

  bool isHeld()
  {
    return mIsHeld.update(lastPressed, mTimeout);
  }

  bool holdStarted();
  
};
}