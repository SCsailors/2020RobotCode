/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Actions/Action.h"
#include "Subsystems/Superstructure.h"

#include <frc/Timer.h>

#include <memory>

class Shoot : public Action {
  std::shared_ptr<Subsystems::Superstructure> mSuperstructure;
  frc::Timer mTimer{};
 public:
  Shoot(){}
  void start()
  {
    std::cout << "Starting Shoot" << std::endl;
    mSuperstructure = Subsystems::Superstructure::getInstance();
    mSuperstructure->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_PRE_EXHAUST_BALL);
    mTimer.Reset();
    mTimer.Start();
    //frc::SmartDashboard::PutString()
    
  }
  void update()
  {
    if (mTimer.Get() > 3.0) //mSuperstructure->isAtDesiredState() ||
    {
      mSuperstructure->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_EXHAUST_BALL);
    }
  }

  void done()
  {
    std::cout << "Finishing Shoot" << std::endl;
  }
  
  bool isFinished()
  {
    return (mSuperstructure->getShooterState() == StateMachines::SuperstructureStateMachine::IDLE || mTimer.Get() > 7.0);
  }

};
