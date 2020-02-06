/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Subsystems/Subsystem.h>

#include <Subsystems/BallPathBottom.h>
#include <Subsystems/BallPathTop.h>
#include <Subsystems/CenteringIntake.h>
#include <Subsystems/Hood.h>
#include <Subsystems/Shooter.h>
#include <Subsystems/Turret.h>

#include <frc/AnalogInput.h>
#include <frc/Solenoid.h>
#include <lib/Util/LatchedBoolean.h>
#include <lib/Geometry/Pose2D.h>
#include <lib/Util/Util.h>

#include <RobotState.h>

#include <lib/Vision/AimingParameters.h>

#include <States/SuperstructureConstants.h>
#include <States/SuperstructureGoal.h>
#include <States/SuperstructureState.h>

#include <memory>

namespace Subsystems {

class Superstructure : public Subsystems::Subsystem {
  static std::shared_ptr<Subsystems::Superstructure> mInstance;
  
  std::shared_ptr<Subsystems::BallPathTop> mBallPathTop;
  std::shared_ptr<Subsystems::BallPathBottom> mBallPathBottom;
  std::shared_ptr<Subsystems::CenteringIntake> mCenteringIntake;
  std::shared_ptr<Subsystems::Hood> mHood;
  std::shared_ptr<Subsystems::Shooter> mShooter;
  std::shared_ptr<Subsystems::Turret> mTurret;
  std::shared_ptr<FRC_7054::RobotState> mRobotState;
  
  frc::Solenoid mWheelieBar{Constants::kPCMID, Constants::kSolenoidID_Wheelie};
  frc::Solenoid mIntake{Constants::kPCMID, Constants::kSolenoidID_Intake};
  
  enum TurretControlModes {ROBOT_RELATIVE, FIELD_RELATIVE, VISION_AIMED, OPEN_LOOP, JOGGING};

  //Current State
  SuperstructureState mCurrentState{};
  //Current Setpoint
  SuperstructureGoal mCurrentSetpoint{};
  //Final Desired State of the Superstructure
  SuperstructureGoal mGoal{};

  std::shared_ptr<Rotation2D> mFieldRelativeGoal = NULL;

  bool mHasTarget = false;
  bool mOnTarget = false;
  int mTrackId = -1;
  
  double mBallPathBottomFeedforwardV = 0.0;
  double mBallPathTopFeedforwardV = 0.0;
  double mCenteringIntakeFeedforwardV = 0.0;
  double mHoodFeedforwardV = 0.0;
  double mShooterFeedforwardV = 0.0;
  double mTurretFeedforwardV = 0.0;
  double mTurretThrottle = 0.0;

  VisionTargeting::AimingParameters mLatestAimingParameters{};
  double mCorrectedRangeToTarget = 0.0;
  bool mEnforceAutoAimMinDistance = false;
  double mAutoAimMinDistance = 12.0;
  
  TurretControlModes mTurretMode = TurretControlModes::ROBOT_RELATIVE;
  Util util{};
 public:
  Superstructure();
  static std::shared_ptr<Subsystems::Superstructure> getInstance();
  
  void OnStart(double timestamp) override;
  void OnLoop(double timestamp) override;
  void OnStop(double timestamp) override;
  
  SuperstructureGoal getGoal();
  SuperstructureState getCurrentState();
  SuperstructureGoal getSetpoint();

  void jogTurret(double delta);
  void setGoal(SuperstructureGoal goal);

  void maybeUpdateGoalFromFieldRelativeGoal(double timestamp);
  void maybeUpdateGoalFromVision(double timestamp);

  void resetAimingParameters();
  double getCorrectedRangeToTarget();
  VisionTargeting::AimingParameters getLatestAimingParameters();
  bool isOnTarget();
  bool getCurrentlyAiming();
  int getTrackId();
  
  void updateCurrentState();

  void setWantAutoAim(std::shared_ptr<Rotation2D> field_to_turret_hint, bool enforce_min_distance, double min_distance);
  void setWantAutoAim(std::shared_ptr<Rotation2D> field_to_turret_hint);
  void setWantRobotRelativeTurret();
  void setWantFieldRelativeTurret(std::shared_ptr<Rotation2D> field_to_turret);

  void setTurretOpenLoop(double throttle);
  void followSetpoint();

  bool isAtDesiredState();
  bool isShooting();
  
  void extendWheelieBar();
  void stowWheelieBar();

  void extendIntake();
  void stowIntake();

};
}