/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "lib/Splines/QuinticHermiteSpline.h"
//util
#include "lib/Util/Units.h"
#include "lib/Util/Util.h"
#include "lib/Util/DriveAssist.h"

//Subsystems
#include "Subsystems/Drive.h"
#include "Subsystems/RobotStateEstimator.h"
#include "Subsystems/Arm.h"
#include "Subsystems/Subsystem.h"
#include "Subsystems/Turret.h"
#include "Subsystems/TurretLimelight.h"

//Paths
#include "Paths/TrajectoryGenerator.h"

//Auto
#include "Auto/AutoModeExecutor.h"
#include "Auto/AutoModeBase.h"

//Loops
#include "Loops/Looper.h"

//Controls
#include "Controls/ControlBoard.h"

//Misc
#include "Kinematics.h"
#include "Constants.h"
#include "RobotState.h"
#include "AutoModeSelector.h"


#include <string>
#include <cmath>
#include <memory>
#include <vector>
using namespace std;

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cameraserver/CameraServer.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  bool isHighGear=false;

//Util

static Util util;
static Units units;
static DriveAssist driveAssist;

//Misc
static Kinematics kinematics;
static Constants constants;
static RobotState robotState;
static AutoModeSelector autoModeSelector;

//Subsystems

static shared_ptr<Subsystems::Drive> drive;
static shared_ptr<Subsystems::RobotStateEstimator> robotStateEstimator;
//static Subsystems::Arm arm;
static shared_ptr<Subsystems::Turret> turret;
static shared_ptr<Subsystems::TurretLimelight> turretlimelight;

//Paths
static TrajectoryGenerator trajectoryGenerator;

//Controls
static ControlBoard controlBoard;

//change once AutoModeSelector and creators are added for the left and right 
shared_ptr<AutoModeExecutor> mAutoModeExecutor;

vector<shared_ptr<Subsystems::Subsystem>> subsystems;

shared_ptr<Looper> mSubsystemLoops;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

#include "Auto/Modes/CharacterizeHighGear.h"
#include "Auto/Modes/PIDTuningMode.h"