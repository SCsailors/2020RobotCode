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

Util Robot::util;
Units Robot::units;
DriveAssist Robot::driveAssist;


Kinematics Robot::kinematics;
Constants Robot::constants;
RobotState Robot::robotState;
AutoModeSelector Robot::autoModeSelector;

shared_ptr<Subsystems::Drive> Robot::drive(new Subsystems::Drive());
shared_ptr<Subsystems::RobotStateEstimator> Robot::robotStateEstimator(new Subsystems::RobotStateEstimator());
//Subsystems::Arm Robot::arm;
shared_ptr<Subsystems::Turret> Robot::turret(new Subsystems::Turret());
shared_ptr<Subsystems::TurretLimelight> Robot::turretlimelight(new Subsystems::TurretLimelight());

TrajectoryGenerator Robot::trajectoryGenerator;

ControlBoard Robot::controlBoard;




void Robot::RobotInit() {
  
  //auto camera0=frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

  //has to be in Initialization because in RobotState constructor, it would cause a crash (RobotState constructor before drive constructor).
  drive->setHeading(make_shared<Rotation2D>());
 
  //shared_ptr<CharacterizeHighGear> characterizeHighGear=make_shared<CharacterizeHighGear>();
  //mAutoModeExecutor= make_shared<AutoModeExecutor>(characterizeHighGear);
  
  //shared_ptr<PIDTuningMode> tuningMode = make_shared<PIDTuningMode>();
  //mAutoModeExecutor= make_shared<AutoModeExecutor>(tuningMode);
  shared_ptr<AutoModeBase> baseMode = make_shared<AutoModeBase>();
  mAutoModeExecutor= make_shared<AutoModeExecutor>(baseMode);
  
  autoModeSelector.updateModeCreator();
  //add subsystem loops to vector
  subsystems.push_back(drive);
  subsystems.push_back(robotStateEstimator);
  subsystems.push_back(turret);
  subsystems.push_back(turretlimelight);
  
  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //generate Trajectories for auto
  
  trajectoryGenerator.generateTrajectories();
 
  //create and start subsystem loops
  mSubsystemLoops=make_shared<Looper>(subsystems);

  mSubsystemLoops->resetSensors();

}


void Robot::RobotPeriodic() {
  robotState.outputToSmartDashboard();
  }

void Robot::DisabledInit(){
  mSubsystemLoops->stopEnabledLoops();
  mSubsystemLoops->startDisabledLoops();
  mAutoModeExecutor->stop();
  autoModeSelector.reset();
  autoModeSelector.updateModeCreator();
}

void Robot::DisabledPeriodic(){
  autoModeSelector.updateModeCreator();

  shared_ptr<AutoModeBase> autoMode = autoModeSelector.getAutoMode(true); //add autofieldstate
  if (autoMode->getID()!= mAutoModeExecutor->getAutoMode()->getID()){
    cout<<"Setting AutoMode to: "+ autoMode->getID()<<endl;
    mAutoModeExecutor->setAutoMode(autoMode);
  }
}

void Robot::AutonomousInit() {
  mSubsystemLoops->resetSensors();
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
 /*
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  //execute auto mode
  */

  mAutoModeExecutor->start();
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
  mAutoModeExecutor->stop();
  
}

void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("ControlBoard/ left", controlBoard.getThrottle());
  frc::SmartDashboard::PutNumber("ControlBoard/ right", controlBoard.getTurn());
  frc::SmartDashboard::PutBoolean("ControlBoard/ isQuickTurn()", controlBoard.getQuickTurn());
  
  double throttle = controlBoard.getThrottle(); 
  double turn = controlBoard.getTurn();  
  shared_ptr<DriveSignal> signal = driveAssist.Drive(throttle, turn, controlBoard.getQuickTurn(),  false);//drive->isHighGear());

  frc::SmartDashboard::PutNumber("Left", signal->getLeft());
  frc::SmartDashboard::PutNumber("Right", signal->getRight());
  frc::SmartDashboard::PutBoolean("brakeMode", signal->getBrakeMode());
  drive->setOpenLoop(driveAssist.Drive(throttle, turn, controlBoard.getQuickTurn(), drive->isHighGear()));
  controlBoard.testHighGearToggle();
  isHighGear=controlBoard.setHighGear();
  drive->setHighGear(isHighGear);
  frc::SmartDashboard::PutBoolean("ControlBoard/ isHighGear", drive->isHighGear());
  frc::SmartDashboard::PutBoolean("ControlBoard/ setHighGear", controlBoard.setHighGear());
  frc::SmartDashboard::PutBoolean("Is AutoShift", drive->mAutoShift);
  //arm->toggleClaw(controlBoard.toggleClaw());
  //arm->togglePickup(controlBoard.toggleBallPickup());
  //arm->setArmPosition(controlBoard.getArmThrottle());

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
