/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Auto/Creators/AutoModeCreator.h"
#include "Auto/AutoModeBase.h"
#include <memory>
using namespace std;

#include "Auto/Modes/CharacterizeHighGear.h"

class DriveCharacterizationCreator : public AutoModeCreator {
  shared_ptr<CharacterizeHighGear> driveCharacterizationMode = make_shared<CharacterizeHighGear>();
 public:
  DriveCharacterizationCreator();
   shared_ptr<AutoModeBase> getStateDependentAutoMode(bool left) override;
};
