/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Util/DriveSignal.h"
#include "lib/Util/Util.h"

#include <cmath>
#include <memory>
using namespace std;

class DriveAssist {
  shared_ptr<Util> util = make_shared<Util>();
  double pi= 3.14159265358979238463;
  //probably need to be tuned
  double kThrottleDeadband=.02;
  double kWheelDeadband= .02;
  
  //Degtermines how fast a wheel traverses the sine curve.
  double kHighWheelNonLinearity=.65;
  double kLowWheelNonLinearity=.5;
  
  double kHighNegInertiaScalar=4.0;

  double kLowNegInertiaThreshold=.65;
  double kLowNegInertiaTurnScalar=3.5;
  double kLowNegInertiaCloseScalar=4.0;
  double kLowNegInertiaFarScalar = 5.0;

  double kHighSensitivity= .65;
  double kLowSensitivity= .65;

  double kQuickStopDeadband=.5;
  double kQuickStopWeight=.1;
  double kQuickStopScalar=5.0;

  double mOldWheel=0.0;
  double mQuickStopAccumulator=0.0;
  double mNegInertiaAccumulator=0.0;
 public:
  DriveAssist();
  //TODO: edit to support tank drive? ... velocity=min(left, right), wheel=(left-right)?
  //Throttle: forward/backward, wheel: how much turn
  shared_ptr<DriveSignal> Drive(double throttle, double wheel, bool isQuickTurn, bool isHighGear);
  double handleDeadband(double val, double deadband);

  
};
