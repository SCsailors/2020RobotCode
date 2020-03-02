/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/OverrideTrajectory.h"
#include "Subsystems/FalconDrive.h"

OverrideTrajectory::OverrideTrajectory() {}

void OverrideTrajectory::start(){
    Subsystems::FalconDrive::getInstance()->overrideTrajectory(true);
    Finished=true;
}
