/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Actions/Action.h"

class CancelShoot : public Action {
 public:
  CancelShoot();
  void start() override;
  void update() override;
  void done() override;
  bool isFinished() override;
};
