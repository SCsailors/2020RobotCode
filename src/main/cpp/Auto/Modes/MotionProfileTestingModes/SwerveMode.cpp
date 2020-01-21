/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Modes/MotionProfileTestingModes/SwerveMode.h"
#include "Robot.h"

SwerveMode::SwerveMode(bool left) {
    mLeft=left;
    swerveForward=make_shared<DriveTrajectory>(Robot::trajectoryGenerator.getTrajectorySet()->DriveForwardSwerveTest->get(mLeft), true);
    swerveReverse=make_shared<DriveTrajectory>(Robot::trajectoryGenerator.getTrajectorySet()->DriveReverseSwerveTest->get(mLeft), false);
    wait = make_shared<WaitAction>(3.0);
}

void SwerveMode::routine(){
     vector<shared_ptr<Action>> actions{swerveForward, wait, swerveReverse};
    shared_ptr<SeriesAction> action = make_shared<SeriesAction>(actions);
    runAction(action);
}

string SwerveMode::getID(){
    return "SwerveMode";
}