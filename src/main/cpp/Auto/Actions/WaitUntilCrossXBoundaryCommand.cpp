/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/WaitUntilCrossXBoundaryCommand.h"

WaitUntilCrossXBoundaryCommand::WaitUntilCrossXBoundaryCommand(double x) {
    mXBoundary=x;
}

bool WaitUntilCrossXBoundaryCommand::isFinished(){
    return Robot::robotState.getLatestFieldToVehicle()->getTranslation()->x()>mXBoundary; 
}