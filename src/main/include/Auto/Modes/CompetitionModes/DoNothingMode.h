/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Auto/Actions/Action.h"
#include "Auto/AutoModeBase.h"

#include <memory>
using namespace std;


class DoNothingMode: public AutoModeBase {
 public:
  DoNothingMode();
  void routine() override;
  string getID() override;
};
