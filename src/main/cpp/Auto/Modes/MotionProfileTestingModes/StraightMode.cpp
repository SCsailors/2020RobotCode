/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Modes/MotionProfileTestingModes/StraightMode.h"
#include "Robot.h"

StraightMode::StraightMode() {
    shared_ptr<DriveTrajectory> forward =make_shared<DriveTrajectory>(Robot::trajectoryGenerator.getTrajectorySet()->DriveForwardStraightTest->get(true), true);
    shared_ptr<WaitAction> wait = make_shared<WaitAction>(3.0);
    shared_ptr<DriveTrajectory> reverse =make_shared<DriveTrajectory>(Robot::trajectoryGenerator.getTrajectorySet()->DriveForwardStraightTest->get(true), false);
}

void StraightMode::routine(){
    
    vector<shared_ptr<Action>> actions{forward, wait, reverse};
    shared_ptr<SeriesAction> action = make_shared<SeriesAction>(actions);
    runAction(action);
    //runAction(make_shared<DriveTrajectory>(Robot::trajectoryGenerator.getTrajectorySet()->DriveForwardStraightTest->get(true), true));


}

string StraightMode::getID(){
    return "StraightMode";
}