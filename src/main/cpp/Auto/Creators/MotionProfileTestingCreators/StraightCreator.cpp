/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Creators/MotionProfileTestingCreators/StraightCreator.h"

StraightCreator::StraightCreator() {}

shared_ptr<AutoModeBase> StraightCreator::getStateDependentAutoMode(bool left){
    return straightMode;
}