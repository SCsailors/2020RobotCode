/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Controls/ControlBoard.h>
#include <Controls/XboxController.h>

#include <lib/Util/MultiTrigger.h>
#include <lib/Util/TimeDelayedBoolean.h>
#include <lib/Util/LatchedBoolean.h>
#include <lib/Util/DelayedBoolean.h>
#include <lib/Util/Util.h>

#include <memory>


namespace ControlBoard{
class GamePadTwoJoysticks : public ControlBoardBase {
  static std::shared_ptr<GamePadTwoJoysticks> mInstance;
  frc::Joystick mLeftJoystick{1};
  frc::Joystick mRightJoystick{2};
  XBoxController mController{Constants::kSingleJoystickPort}; 
  double mDPadDelay = .2;
  bool wantsHighGear = false;
  bool wantsManual = true;
  int prev_dpad = -1;
  double mTurretCardinalOutput = 0.0;
  bool dPadValid = false;
  Utility::LatchedBoolean mDPadUpdate{};

  Utility::MultiTrigger LT_Multi{Constants::kJoystickHoldTime};
  Utility::MultiTrigger RT_Multi{Constants::kJoystickHoldTime};
  Utility::MultiTrigger LB_Multi{Constants::kJoystickHoldTime};
  Utility::MultiTrigger RB_Multi{Constants::kJoystickHoldTime};

  Utility::LatchedBoolean A{};
  Utility::LatchedBoolean B{};
  Utility::MultiTrigger Start{Constants::kJoystickHoldTime};
  Utility::LatchedBoolean Y{};
  Utility::LatchedBoolean X{};
  Utility::LatchedBoolean Back{};
  Utility::LatchedBoolean HighGear{};
  Utility::LatchedBoolean ManualShiftToggle{};

  Utility::TimeDelayedBoolean mDPadValid{};

 public:
  GamePadTwoJoysticks();
  static std::shared_ptr<GamePadTwoJoysticks> getInstance();
  double getThrottle() override;
  double getTurn() override;
  bool getQuickTurn() override;
  bool getWantsHighGear() override;
  bool getDriveStraight() override;

  bool getShoot() override;
  double getBallShootCount(bool preshoot) override;
  double shootCount = 5.0;

  bool getWheel() override;
  bool getWantsRotation() override;
  
  bool getClimber() override;
  bool getClimbRun() override;

  bool getIntake() override;
  bool getCancel() override;
  
  bool isTurretJogging() override;
  std::shared_ptr<Rotation2D> getTurretCardinal() override;
  bool getValidTurretCardinal() override;
  
  bool getAutoAim() override;
  bool getFieldRelative() override;
  bool AutoAim = true;
  bool FieldRelative = false;


  void reset();

  double getHood() override;
  double getShooter() override;
  double getBallPath() override;
  double getTurretJog() override;
  
  bool getDriveShifterManual() override;
  
  bool getCloseShoot() override;
  bool getLineShoot() override;
  

  int i = 0;
  int j = 0;
  int k = 0;
  int l = 0;
  int m = 0;
  int n = 0;
  int o = 0;
  int p = 0;
};
}