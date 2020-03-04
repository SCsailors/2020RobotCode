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
  std::shared_ptr<Shoot> mShoot;
  std::shared_ptr<WaitAction> mWait;
  std::shared_ptr<CancelShoot> mCancelShoot;
  std::shared_ptr<OpenLoopDrive> mOpenLoopDrive;
  std::shared_ptr<ParallelAction> mParallel;
  std::shared_ptr<SeriesAction> mSeries;
 public:
  SimpleMode()
  {
    mShoot = std::make_shared<Shoot>();
    mWait = std::make_shared<WaitAction>(8.0);
    mCancelShoot = std::make_shared<CancelShoot>();
    mOpenLoopDrive = std::make_shared<OpenLoopDrive>(-.75, -.75, 2.0, false);

    std::vector<std::shared_ptr<Action>> parallel{mShoot, mWait};
    mParallel = std::make_shared<ParallelAction>(parallel);
    
    std::vector<std::shared_ptr<Action>> series{mParallel, mCancelShoot, mOpenLoopDrive};
    mSeries = std::make_shared<SeriesAction>(series);
  }
  void routine();
  std::string getID();
};
