/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Actions/Action.h"
#include "Subsystems/Superstructure.h"

class ToggleIntake : public Action {
  bool extendIntake = false;
 public:
  ToggleIntake(bool extend): extendIntake(extend){}
  void start()
  {
    Subsystems::Superstructure::getInstance()->setWantedActionIntake(extendIntake? StateMachines::SuperstructureStateMachine::WantedAction::WANTED_INTAKE_BALL : StateMachines::SuperstructureStateMachine::WantedAction::WANTED_HAVE_BALLS);
  }
  void update(){}
  void done(){}
  bool isFinished(){return true;}
};
