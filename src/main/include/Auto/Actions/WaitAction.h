/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Action.h"
#include <iostream>
using namespace std;

#include "frc/Timer.h"



class WaitAction: public Action {
  double mTimeToWait=0.0;
  double mStartTime=0.0;
 public:
  WaitAction(double time_to_wait);

  bool isFinished();
  void start();
  void update(){}
  void done(){cout<<"Ending wait action"<<endl;}
};

  