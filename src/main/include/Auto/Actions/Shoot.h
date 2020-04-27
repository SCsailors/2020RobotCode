/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Actions/Action.h"
#include "Subsystems/Superstructure.h"

#include <memory>

class Shoot : public Action {
 std::shared_ptr<Subsystems::Superstructure> mSuperstructure;
 public:
  Shoot(){}
  void start()
  {
    mSuperstructure = Subsystems::Superstructure::getInstance();
    mSuperstructure->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_PRE_EXHAUST_BALL);
  }
  void update()
  {
    if (mSuperstructure->isAtDesiredState())
    {
      mSuperstructure->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_EXHAUST_BALL);
    }
  }
  void done()
  {

  }
  bool isFinished()
  {
    return mSuperstructure->getShooterState() == StateMachines::SuperstructureStateMachine::IDLE;
  }

};
