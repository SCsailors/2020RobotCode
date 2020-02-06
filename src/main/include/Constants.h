/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Subsystems/Limelight.h"
#include "Subsystems/ServoMotorSubsystem.h"
#include <lib/Geometry/Pose2D.h>
#include <memory>

class Constants {
 public:
  static std::shared_ptr<Subsystems::LimelightConstants> kTurretLimelightConstants;
  static std::shared_ptr<Subsystems::LimelightConstants> kIntakeLimelightConstants;
  
  static std::shared_ptr<Subsystems::TalonConstants> kBallPathBottomConstants;
  static std::shared_ptr<Subsystems::TalonConstants> kBallPathTopConstants;
  static std::shared_ptr<Subsystems::TalonConstants> kCenteringIntakeConstants;
  static std::shared_ptr<Subsystems::TalonConstants> kHoodConstants;
  static std::shared_ptr<Subsystems::TalonConstants> kShooterConstants;
  static std::shared_ptr<Subsystems::SparkMaxConstants> kTurretConstants;
  
  Constants();
  constexpr static double kPI = 3.14159265358979238463;

  constexpr static double kRadsToDegrees = 180.0/kPI;
  constexpr static double kDegreesToRads = kPI/180.0;
  
  constexpr static double kCameraFrameRate = 90.0;
  constexpr static double kImageCaptureLatency = kCameraFrameRate/1000.0;
  
  constexpr static double kLimelightWidth = 320.0;
  constexpr static double kLimelightHeight = 240.0;
  
  constexpr static double kMaxTrackerDistance = 648.0; //tune
  constexpr static double kMaxTrackAgeNotTracking = .1;
  constexpr static double kMaxGoalTrackSmoothingTime = .5;
  constexpr static double kMaxGoalTrackAge = 2.5;
  constexpr static double kTrackStabilityWeight = 0.0;
  constexpr static double kTrackAgeWeight = 10.0;
  constexpr static double kTrackSwitchingWeight = 100.0;
  //ROBOT PHYSICAL CONSTANTS-need tuning: these set values are just to give an estimate
  constexpr static int kLongCANTimeoutMs = 100;
  constexpr static int kCANTimeoutMs = 10;
  constexpr static int kMotionProfileSlot = 0;
  constexpr static int kPositionPIDSlot = 1;
  constexpr static int kVelocityPIDSlot = 2;
  constexpr static int kMotionMagicPIDSlot = 3; //motion magic = smart motion;

  constexpr static double kBallDetectPassTime = .5;
  constexpr static double kBallDetectHoldTime = 1.0;


  constexpr static int kPCMID = 1;
  //Solenoid Ports
  constexpr static int kSolenoidID_WheelOfFortune = 0;
  constexpr static int kSolenoidID_Wheelie = 1;
  constexpr static int kSolenoidID_Intake = 2;
  //DIO Ports
  constexpr static int kDIO_FirstBreak = 0;
  constexpr static int kDIO_FirstMake = 1;
  constexpr static int kDIO_LastBreak = 2;
  constexpr static int kDIO_LastMake = 3;
  //AIO Ports


  //Max drive Velocity and Acceleration
  constexpr static double kDriveMaxVelocity=75.0;//max recorded 88 inches/second
  constexpr static double kDriveMaxAcceleration=30.0;//max recorded 200 inches/second^2
  constexpr static double kDriveMaxCentripetalAcceleration=100.0;
  constexpr static double kDriveMaxVoltage=9.0;
  //Wheels
  constexpr static double kDriveWheelTrackWidthInches=27.5;
  constexpr static double kDriveWheelDiameterInches=6.25;//6.25;
  constexpr static double kDriveWheelRadiusInches=kDriveWheelDiameterInches/2.0;
  constexpr static double kTrackScrubFactor=1.1375;

  //Tuned dynamics...well not totally tuned yet (top 3 need to be)
  constexpr static double kRobotLinearInertia=60.0; //kg
  constexpr static double kRobotAngularInertia=10.0; //kg m^2
  constexpr static double kRobotAngularDrag=12.0; //N*m/(rad/sec)
  constexpr static double kDriveVIntercept=0.142557 ; // V
  constexpr static double kDriveKv=0.3944122*1.055; //V per rad/s
  constexpr static double kDriveKa =0.012; //V per rad/s^2 Ex: .012

  //Geometry
  constexpr static double kCenterToFrontBumperDistance= 38.25/2.0; //in
  constexpr static double kCenterToRearBumperDistance=38.25/2.0;
  constexpr static double kCenterToSideBumperDistance= 33.75/2.0;

  //Gearing and mechanical constexprants
  constexpr static double kDriveDownShiftVelocity= 5.75*12.0; //inches per second
  constexpr static double kDriveDownShiftAngularVelocity= kPI/2.0;
  constexpr static double kDriveUpShiftVelocity= 11.0*12.0; //inches per second


  //drive velocity loop (low Gear) TODO TUNE
  constexpr static double kDriveLowGearVelocityKp =0.0;//5e-5;
  constexpr static double kDriveLowGearVelocityKi = 0.0; //1e-6;
  constexpr static double kDriveLowGearVelocityKiZone = 0.0;
  constexpr static double kDriveLowGearVelocityKd = 0.0;
  constexpr static double kDriveLowGearVelocityKf =0.0;//.000015;
  constexpr static double kAcceleration = 200.0;

  //Drive Inches/Rotation
  constexpr static double kDriveHighGearIPR=100.0/38.67;//100.0/34.5;
  constexpr static double kDriveLowGearIPR=100.0/103.6;//100.0/93.5;
  constexpr static double kMotionPlannerLookAhead=.005;

};