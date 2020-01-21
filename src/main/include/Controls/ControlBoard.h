/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Joysticks.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <memory>
using namespace std;

class ControlBoard {
  shared_ptr<Joysticks> joysticks= make_shared<Joysticks>();
  bool stop=false;

  bool high=false;
  bool prev_high=false;
  bool prev_HighGear=false;

  bool claw=false;
  bool prevClaw=false;
  bool prev_Claw=false;

  bool pickup=false;
  bool prevPickup=false;
  bool prev_Pickup=false;
  
  
  
  bool highGear=false;
  int hGear=0;
  int lGear=0;
 public:
  ControlBoard();
  double getThrottle();
  double getTurn();
  bool getQuickTurn();
  bool overrideTrajectory();
  bool setHighGear();
  double getArmThrottle();
  bool toggleClaw();
  bool toggleBallPickup();

  void testHighGearToggle();

};
