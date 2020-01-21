/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Action.h"
#include "Robot.h"

class OverrideTrajectory: public Action {
  bool Finished=false;
 public:
  OverrideTrajectory();
  void start();
  void update(){}
  void done(){}
  bool isFinished(){return Finished;}
};
