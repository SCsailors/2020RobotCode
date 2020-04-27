/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Creators/MotionProfileTestingCreators/SwerveCreator.h"

SwerveCreator::SwerveCreator(bool left) {
    swerveMode=make_shared<SwerveMode>(left);
}

shared_ptr<AutoModeBase> SwerveCreator::getStateDependentAutoMode(bool left){
    return swerveMode;
}
