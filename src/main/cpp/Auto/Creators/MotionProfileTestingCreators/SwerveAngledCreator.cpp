/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Creators/MotionProfileTestingCreators/SwerveAngledCreator.h"

SwerveAngledCreator::SwerveAngledCreator(bool left) {
    swerveAngledMode=make_shared<SwerveAngledMode>(left);
}

shared_ptr<AutoModeBase> SwerveAngledCreator::getStateDependentAutoMode(bool left){
    return swerveAngledMode;
}