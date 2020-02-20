/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <iostream>
#include <memory>

class LEDState {
 public:
  double PWM = 0.99; //off
  LEDState(double pwm): PWM(pwm) {}

  void copyFrom(LEDState ledState)
  {
    this->PWM = ledState.PWM;
  }
};

class TimedLEDState {
 public:
  TimedLEDState(){}
  virtual void  getCurrentLEDState(LEDState &desiredState, double timestamp)
  {
    std::cout << "Error: Calling TimedLEDState.getCurrentLEDState() instead of inherited! "<<std::endl;
  }

};

class BlinkingLEDState : public TimedLEDState 
{
  public:
    LEDState mStateOne{0.99};
    LEDState mStateTwo{.99};
    double mDuration = 0.0;
    BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration);
    void getCurrentLEDState(LEDState &desiredState, double timestamp) override;
  
};

class StaticLEDState : public TimedLEDState
{
  public:
    LEDState mStaticState{.99};
    StaticLEDState(LEDState staticState);
    void getCurrentLEDState(LEDState &desiredState, double timestamp) override;
  
    //Global  
    
};

class TimedLEDStates
{
 public:
  TimedLEDStates();
  
    //add specific pwm colors and actions here as static LEDState.
  //http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  
  //Global colors
  const LEDState Off{.99};
  const LEDState Fault{0.59};
  const LEDState Start{-.99};
  const LEDState End{-.95};
  const LEDState Zeroed{.73};
  const LEDState ReadyToHang{-.49};
  const LEDState Hanging{-.41};

  //Intaking colors
  const LEDState Intaking{0.79};
  const LEDState StowingIntake{0.69};
  //Five ball colors
  const LEDState OneBall{0.57};
  const LEDState TwoBalls{0.65};
  const LEDState ThreeBalls{0.69};
  const LEDState FourBalls{0.73};
  const LEDState FiveBalls{0.81};
  
  //Shooting
  const LEDState PreShooting{0.83};
  const LEDState Shooting{0.69};

  //Wheel
  const LEDState WheelStart{.91};
  const LEDState WheelPosition{0.71};
  const LEDState WheelRotation{0.61};
  const LEDState WheelComplete{0.95};



  constexpr static double kFastBlink = .1;
  constexpr static double kMedBlink = .25;
  constexpr static double kSlowBlink = 1.0;

  std::shared_ptr<BlinkingLEDState> kFault;
  std::shared_ptr<BlinkingLEDState> kJustZeroed;

  //Intaking and stowing ready to progress
  std::shared_ptr<BlinkingLEDState> kIntakingOne;
  std::shared_ptr<BlinkingLEDState> kIntakingTwo;
  std::shared_ptr<BlinkingLEDState> kIntakingThree;
  std::shared_ptr<BlinkingLEDState> kIntakingFour;
  std::shared_ptr<BlinkingLEDState> kIntakingFive;

  std::shared_ptr<BlinkingLEDState> kStowingZero;
  std::shared_ptr<BlinkingLEDState> kStowingOne;
  std::shared_ptr<BlinkingLEDState> kStowingTwo;
  std::shared_ptr<BlinkingLEDState> kStowingThree;
  std::shared_ptr<BlinkingLEDState> kStowingFour;
  std::shared_ptr<BlinkingLEDState> kStowingFive;

  //Shooter ready to progress
  std::shared_ptr<BlinkingLEDState> kShooterReady;
  std::shared_ptr<BlinkingLEDState> kShootingComplete;

  //Wheel ready to progress 
  std::shared_ptr<BlinkingLEDState> kWheelReady; 
  std::shared_ptr<BlinkingLEDState> kWheelComplete;

  std::shared_ptr<BlinkingLEDState> kPreHangComplete; 
  std::shared_ptr<BlinkingLEDState> kHangComplete;

  std::shared_ptr<StaticLEDState> kStaticOff;
  std::shared_ptr<StaticLEDState> kRobotZeroed;
  std::shared_ptr<StaticLEDState> kIdle;
  std::shared_ptr<StaticLEDState> kEnd;

  std::shared_ptr<StaticLEDState> kIntakeOut;
  std::shared_ptr<StaticLEDState> kStowing;
    //Five Balls
  std::shared_ptr<StaticLEDState> kOneBall;
  std::shared_ptr<StaticLEDState> kTwoBalls;
  std::shared_ptr<StaticLEDState> kThreeBalls;
  std::shared_ptr<StaticLEDState> kFourBalls;
  std::shared_ptr<StaticLEDState> kFiveBalls;

  std::shared_ptr<StaticLEDState> kPreShooting;
  std::shared_ptr<StaticLEDState> kShooting;

  std::shared_ptr<StaticLEDState> kWheelStarted;
  std::shared_ptr<StaticLEDState> kWheelPosition;
  std::shared_ptr<StaticLEDState> kWheelRotation;

  std::shared_ptr<StaticLEDState> kPreHangStarted;
  std::shared_ptr<StaticLEDState> kClimbing;
};