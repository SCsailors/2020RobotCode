/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Auto/Creators/AutoModeCreator.h"
#include "Auto/AutoModeBase.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
using namespace std;

class AutoModeSelector {
  shared_ptr<AutoModeCreator> mCreator= make_shared<AutoModeCreator>();
 public:
 enum StartingPosition{LEFT, CENTER, RIGHT, NONE0};
 StartingPosition mStartingPosition;
 string mPosition="NONE";

 enum Mode{COMPETITION, MOTION_PROFILE_TESTING, PID_TUNING, DRIVE_CHARACTERIZATION, NONE1};
 Mode mCachedMode=Mode::NONE1;
 string mMode="NONE";

 enum CompetitionMode{DO_NOTHING, NONE2, SIMPLE, LINE_SHOOT};
 CompetitionMode mCachedCompetitionMode=CompetitionMode::NONE2;

 enum TestingMode{STRAIGHT_TEST, SWERVE_TEST, SWERVE_ANGLED_TEST, NONE3};
 TestingMode mCachedTestingMode=TestingMode::NONE3;

  AutoModeSelector();
  void updateModeCreator();
  void reset();
  shared_ptr<AutoModeCreator> getCreatorForParams(Mode mode, CompetitionMode competition, TestingMode testing, StartingPosition position);
  shared_ptr<AutoModeBase> getAutoMode(bool left);

  frc::SendableChooser<Mode> mModeChooser;
  frc::SendableChooser<StartingPosition> mStartPositionChooser;
  frc::SendableChooser<CompetitionMode> mCompetitionModeChooser;
  frc::SendableChooser<TestingMode> mTestingModeChooser;

};
