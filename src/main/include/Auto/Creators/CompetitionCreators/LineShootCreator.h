/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Creators/AutoModeCreator.h"

#include "Auto/Modes/CompetitionModes/LineShootMode.h"

#include <memory>

class LineShootCreator : public AutoModeCreator {
  std::shared_ptr<LineShootMode> mLineShootMode = std::make_shared<LineShootMode>();
 public:
  LineShootCreator();
  std::shared_ptr<AutoModeBase> getStateDependentAutoMode(bool left)
  {
    return mLineShootMode;
  }
};
