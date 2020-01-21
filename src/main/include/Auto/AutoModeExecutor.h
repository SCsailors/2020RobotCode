/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/AutoModeBase.h"

#include "frc/Notifier.h"

#include <memory>
#include <iostream>
using namespace std;
 
class AutoModeExecutor {
  shared_ptr<AutoModeBase> m_auto_mode;
  
 public:
  AutoModeExecutor(shared_ptr<AutoModeBase> new_auto_mode);
  void start();
  void stop();
  shared_ptr<AutoModeBase> getAutoMode();
  void setAutoMode(shared_ptr<AutoModeBase> autoMode);

};
