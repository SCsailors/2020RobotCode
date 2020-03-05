/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Actions/Action.h"

#include "Subsystems/Superstructure.h"

#include <iostream>

class CancelShoot : public Action {
 public:
  CancelShoot(){}
  void start()
  {
    std::cout << "Starting Cancel Shoot" << std::endl;
    Subsystems::Superstructure::getInstance()->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_IDLE);
  }

  void update(){}
  void done(){}
  bool isFinished()
  {
    return true;
  }
};
