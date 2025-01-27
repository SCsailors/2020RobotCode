/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Action.h"
#include "Robot.h"



class WaitUntilCrossXBoundaryCommand: public Action {
  double mXBoundary=0.0;
 public:
  WaitUntilCrossXBoundaryCommand(double x);
  bool isFinished();
  void start(){}
  void update(){}
  void done(){}


};
  