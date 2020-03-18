/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/AutoModeBase.h"

#include "Auto/Actions/CancelShoot.h"
#include "Auto/Actions/SeriesAction.h"
#include "Auto/Actions/Shoot.h"
#include "Auto/Actions/ParallelAction.h"
#include "Auto/Actions/WaitAction.h"
#include "Auto/Actions/OpenLoopDrive.h"

#include <memory>

class SimpleMode : public AutoModeBase{
  std::shared_ptr<OpenLoopDrive> mOpenLoopDrive;
  
 public:
  SimpleMode()
  {
    mOpenLoopDrive = std::make_shared<OpenLoopDrive>(.5, .5, 2.0, false);
  }
  void routine();
  std::string getID();
};
