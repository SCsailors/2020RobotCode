/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/WaitUntilCrossXBoundaryCommand.h"

#include <RobotState.h>

WaitUntilCrossXBoundaryCommand::WaitUntilCrossXBoundaryCommand(double x) {
    mXBoundary=x;
}

bool WaitUntilCrossXBoundaryCommand::isFinished(){
    return FRC_7054::RobotState::getInstance()->getLatestFieldToVehicle()->getTranslation()->x()>mXBoundary; 
}