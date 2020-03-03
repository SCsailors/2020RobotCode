/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Auto/Actions/Action.h"

#include "Auto/Actions/ParallelAction.h"
#include "Auto/Actions/SeriesAction.h"
#include "Auto/Actions/WaitAction.h"
#include "Auto/Actions/Shoot.h"
#include "Auto/Actions/OpenLoopDrive.h"
#include "Auto/Actions/CancelShoot.h"

#include "Auto/AutoModeBase.h"

#include <memory>

class SimpleMode : public AutoModeBase {
  std::shared_ptr<ParallelAction> mParallel;
  std::shared_ptr<SeriesAction> mSeries;
  std::shared_ptr<Shoot> mShoot;
  std::shared_ptr<OpenLoopDrive> mOpenLoopDrive;
  std::shared_ptr<CancelShoot> mCancelShoot;
  std::shared_ptr<WaitAction> mWait;
 public:
  SimpleMode();
  void routine() override;
  std::string getID() override;
};
