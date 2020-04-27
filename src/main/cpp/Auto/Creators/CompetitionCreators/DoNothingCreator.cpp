/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Creators/CompetitionCreators/DoNothingCreator.h"

DoNothingCreator::DoNothingCreator() {}

shared_ptr<AutoModeBase> DoNothingCreator::getStateDependentAutoMode(bool left){
    return doNothingMode;
}