/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Auto/Creators/AutoModeCreator.h"

#include "Auto/Modes/CompetitionModes/SimpleMode.h"

#include <memory>

class SimpleCreator : public AutoModeCreator{
  std::shared_ptr<SimpleMode> mSimpleMode = std::make_shared<SimpleMode>();
 public:
  SimpleCreator();
  std::shared_ptr<AutoModeBase> getStateDependentAutoMode(bool left) override;
};
