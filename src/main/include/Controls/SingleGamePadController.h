/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Controls/ControlBoard.h"
#include "Controls/XBoxController.h"
#include "Constants.h"

#include "lib/Util/LatchedBoolean.h"
#include "lib/Util/MultiTrigger.h"
#include "lib/Util/TimeDelayedBoolean.h"
#include "lib/Util/DelayedBoolean.h"

#include <memory>

namespace ControlBoard {

class SingleGamePadController : public ControlBoard::ControlBoardBase {
  static std::shared_ptr<SingleGamePadController> mInstance;
  TurretCardinalEnum mLastCardinal = TurretCardinalEnum::NONE;
  Utility::DelayedBoolean mDPadValid;
  double mDPadDelay = .02;
  bool wantsHighGear = false;
  bool wantsManual = true;
  XBoxController mController{Constants::kSingleJoystickPort}; 
  
  Utility::MultiTrigger LT_Multi{Constants::kJoystickHoldTime};
  Utility::MultiTrigger RT_Multi{Constants::kJoystickHoldTime};
  Utility::MultiTrigger LB_Multi{Constants::kJoystickHoldTime};
  Utility::MultiTrigger RB_Multi{Constants::kJoystickHoldTime};

  Utility::LatchedBoolean A{};
  Utility::LatchedBoolean B{};
  Utility::LatchedBoolean X{};
  Utility::LatchedBoolean Y{};
  Utility::LatchedBoolean Start{};
  Utility::LatchedBoolean Back{};

 public:
  SingleGamePadController();
  static std::shared_ptr<SingleGamePadController> getInstance();
  double getThrottle() override;
  double getTurn() override;
  bool getQuickTurn() override;
  bool getWantsHighGear() override;

  bool getShoot() override;
  bool getWheel() override;
  bool getWantsRotation() override;
  bool getClimber() override;
  bool getIntake() override;
  bool getCancel() override;
  double getTurretJog() override;
  bool isTurretJogging() override;
  TurretCardinal getTurretCardinal() override;
  void reset();
  //bool getAutoAim() override;
  double getBallShootCount(bool preshoot) override;
  double shootCount = 5.0;

  double getHood() override;
  bool getDriveShifterManual() override;

  int i = 0;
  int j = 0;
  int k = 0;
  int l = 0;
  int m = 0;
  int n = 0;
  int o = 0;
  int p = 0;

};
}