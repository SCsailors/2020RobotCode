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

  NavX.Reset();
  NavX.ZeroYaw();
  
  //add subsystem loops to vector
  
  subsystems.push_back(mLimelightManager);
  //create and start subsystem loops
  mSubsystemLoops=make_shared<Looper>(subsystems);

  mSubsystemLoops->resetSensors();
  std::cout<<"reset sensors"<<std::endl;
}


void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("NavX Heading", NavX.GetFusedHeading());
}

void Robot::DisabledInit(){
  //mSuperstructure->setWantRobotRelativeTurret(Rotation2D::fromDegrees(0.0));
  //mLimelightManager->setAllLEDS(Subsystems::Limelight::LedMode::OFF);
  mSubsystemLoops->stopEnabledLoops();
  mSubsystemLoops->startDisabledLoops();
}

void Robot::DisabledPeriodic(){

}

void Robot::AutonomousInit() {
  //mSuperstructure->setWantAutoAim(Rotation2D::fromDegrees(0.0));
  mLimelightManager->setAllLEDS(Subsystems::Limelight::LedMode::ON);
  mSubsystemLoops->resetSensors();
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
}

void Robot::AutonomousPeriodic() 
{
  mLimelightManager->setAllLEDS(Subsystems::Limelight::LedMode::ON);
  /*
  //simple drive auto
  if (mAutoTimer.Get() < 2.0)
  {
    std::shared_ptr<DriveSignal> signal = std::make_shared<DriveSignal>(.3, .3);
    mDrive->setOpenLoop(signal);
  }
  else
  {
    std::shared_ptr<DriveSignal> signal = std::make_shared<DriveSignal>(0.0,0.0);
    mDrive->setOpenLoop(signal);
  }
  */
  /*
  //Qual 15: Enginerds
  if (mAutoTimer.Get() > 8.0 && mAutoTimer.Get() < 11.0)
  {
    mBallPathTop->setOpenLoop(-.8);
    mSuperstructure->setBallPathManual(true);
  } 

  if (mAutoTimer.Get() > 12.0 && mAutoTimer.Get() < 15.0)
  {
    mBallPathTop->setOpenLoop(0.0);
    mSuperstructure->setBallPathManual(false);
    std::shared_ptr<DriveSignal> signal = std::make_shared<DriveSignal>(-.3, -.7);
    mDrive->setOpenLoop(signal);
  } else
  {
    std::shared_ptr<DriveSignal> signal = std::make_shared<DriveSignal>(0.0,0.0);
    mDrive->setOpenLoop(signal);
  }
  */
}

void Robot::TeleopInit() {
  //mSuperstructure->setWantAutoAim(Rotation2D::fromDegrees(0.0));
  mLimelightManager->setAllLEDS(Subsystems::Limelight::LedMode::ON);
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
  
}

void Robot::TeleopPeriodic() {
  mLimelightManager->setAllLEDS(Subsystems::Limelight::LedMode::ON);
}


void Robot::TestPeriodic() 
{
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
