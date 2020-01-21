/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Creators/PIDTuningCreator.h"

PIDTuningCreator::PIDTuningCreator() {}

shared_ptr<AutoModeBase> PIDTuningCreator::getStateDependentAutoMode(bool left){
    return pidTuning;
}