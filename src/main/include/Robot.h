/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Constants.h"

//ALWAYS include in .cpp file, not .h file
#include "lib/Splines/QuinticHermiteSpline.h"
//util
#include "lib/Util/Units.h"
#include "lib/Util/Util.h"
#include "lib/Util/DriveAssist.h"
#include "lib/Util/LatchedBoolean.h"
#include "lib/Util/ButtonState.h"
#include "lib/Util/MultiTrigger.h"
#include "lib/Util/LatchedBoolean.h"

//Subsystems
//#include "Subsystems/Drive.h"
#include "Subsystems/FalconDrive.h"
#include "Subsystems/RobotStateEstimator.h"
#include "Subsystems/Subsystem.h"

#include "Subsystems/BallPathTop.h"
#include "Subsystems/CenteringIntake.h"
#include "Subsystems/Hood.h"
#include "Subsystems/Infrastructure.h"
#include "Subsystems/LED.h"
#include "Subsystems/LimelightManager.h"
#include "Subsystems/Shooter.h"
#include "Subsystems/Superstructure.h"
#include "Subsystems/WinchSystem.h"

//Paths
#include "Paths/TrajectoryGenerator.h"

//Auto
#include "Auto/AutoModeExecutor.h"
#include "Auto/AutoModeBase.h"

//Loops
#include "Loops/Looper.h"

//Controls
#include "Controls/ControlBoard.h"
#include "Controls/SingleGamePadController.h"
#include "Controls/TwoGamePadControllers.h"
#include "Controls/GamePadTwoJoysticks.h"

//Misc
#include "Kinematics.h"
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

std::shared_ptr<FRC_7054::RobotState> mRobotState;
static AutoModeSelector autoModeSelector;

//Subsystems

//static shared_ptr<Subsystems::Drive> drive;
std::shared_ptr<Subsystems::FalconDrive> mDrive;
static shared_ptr<Subsystems::RobotStateEstimator> robotStateEstimator;

std::shared_ptr<Subsystems::BallPathTop> mBallPathTop;
std::shared_ptr<Subsystems::CenteringIntake> mCenteringIntake;
std::shared_ptr<Subsystems::Hood> mHood;
std::shared_ptr<Subsystems::Infrastructure> mInfrastructure;
std::shared_ptr<Subsystems::LED> mLED;
std::shared_ptr<Subsystems::LimelightManager> mLimelightManager;
std::shared_ptr<Subsystems::Shooter> mShooter;
std::shared_ptr<Subsystems::Superstructure> mSuperstructure;
std::shared_ptr<Subsystems::Turret> mTurret;
std::shared_ptr<Subsystems::WinchSystem> mClimber;

//Controls
std::shared_ptr<ControlBoard::ControlBoardBase> mControlBoard;

//Paths
static TrajectoryGenerator trajectoryGenerator;

//change once AutoModeSelector and creators are added for the left and right 
shared_ptr<AutoModeExecutor> mAutoModeExecutor;

vector<shared_ptr<Subsystems::Subsystem>> subsystems;

shared_ptr<Looper> mSubsystemLoops;

void manualControl();
void TestControl();

 private:

  bool preshoot = false;
  bool shooting = false;
  double pre_shoot_start_time = 0.0;
  double shoot_start_time = 0.0;
  bool jogging = false;
  bool intake_extended = false;
  double extended_start_time = 0.0;
  double prev_ball = 0.0;
  bool prev_controller_one = true;
  bool controller_one = true;
  bool ballPathToggle = false;

  bool pre_climb = false;
  bool climbing = false;
  bool climbing_finished = false;

  Utility::LatchedBoolean manualDriveShifter{};
  Utility::LatchedBoolean autoDriveShifter{false};

  double lineHood = 46.0;
  double lineShooter = 90.0;

  double closeHood = 0.0;
  double closeShooter = 30.0;
  bool isShootClose = false;

  int i = 0;
  int j = 0;
  int k = 0;
  int l = 0;
  int m = 0;
  int n = 0;
  int o = 0;
  int p = 0;

};

#include "Auto/Modes/CharacterizeHighGear.h"
#include "Auto/Modes/PIDTuningMode.h"