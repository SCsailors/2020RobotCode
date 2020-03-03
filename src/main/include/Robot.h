/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Constants.h"

//#include "Subsystems/Drive.h"
#include "Subsystems/Subsystem.h"

#include "Subsystems/LimelightManager.h"

//Loops
#include "Loops/Looper.h"

#include "RobotState.h"


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

//Util
std::shared_ptr<FRC_7054::RobotState> mRobotState;

//Subsystems

//static shared_ptr<Subsystems::Drive> drive;
std::shared_ptr<Subsystems::LimelightManager> mLimelightManager;


vector<shared_ptr<Subsystems::Subsystem>> subsystems;

shared_ptr<Looper> mSubsystemLoops;

 private:

};
