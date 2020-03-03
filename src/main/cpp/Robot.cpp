/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include "lib/Util/CSVWriter.h"

void Robot::RobotInit() {

  mRobotState = FRC_7054::RobotState::getInstance();
  
  mLimelightManager = Subsystems::LimelightManager::getInstance();

  mRobotState->reset();

  mLimelightManager->setAllLEDS(Subsystems::Limelight::LedMode::ON);
  
  //add subsystem loops to vector
  
  subsystems.push_back(mLimelightManager);

  //create and start subsystem loops
  mSubsystemLoops=make_shared<Looper>(subsystems);

  mSubsystemLoops->resetSensors();
  std::cout<<"reset sensors"<<std::endl;
}


void Robot::RobotPeriodic() {
  
}

void Robot::DisabledInit(){
  mSubsystemLoops->stopEnabledLoops();
  mSubsystemLoops->startDisabledLoops();
}

void Robot::DisabledPeriodic(){

}

void Robot::AutonomousInit() {
  mSubsystemLoops->resetSensors();
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
  
}

void Robot::TeleopPeriodic() {
 
}


void Robot::TestPeriodic() 
{
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
