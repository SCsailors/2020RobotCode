/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "States/LEDState.h"

class TimedLEDState {
 public:
  TimedLEDState(){}
  virtual void  getCurrentLEDState(LEDState &desiredState, double timestamp){}
};

class BlinkingLEDState : public TimedLEDState 
{
  public:
    LEDState mStateOne{0.99};
    LEDState mStateTwo{.99};
    double mDuration = 0.0;
    BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration);
    void getCurrentLEDState(LEDState &desiredState, double timestamp) override;

  //add blinking LEDState as static
};

class StaticLEDState : public TimedLEDState
{
  public:
    LEDState mStaticState{.99};
    StaticLEDState(LEDState staticState, double timestamp);
    void getCurrentLEDState(LEDState &desiredState, double timestamp) override;
  
  //add static LEDState as static
    const static LEDState kStaticOff;
};