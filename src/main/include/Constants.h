/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <lib/Geometry/Pose2D.h>
#include <memory>

//#define CompetitionBot 1
#define Not_Implemented 1

namespace Constants {
 
 
  const double kPI = 3.14159265358979238463;

  const double kRadsToDegrees = 180.0/kPI;
  const double kDegreesToRads = kPI/180.0;
  
  const double kCameraFrameRate = 90.0;
  const double kImageCaptureLatency = kCameraFrameRate/1000.0;
  
  const double kLimelightWidth = 320.0;
  const double kLimelightHeight = 240.0;
  
  const double kMaxTrackerDistance = 648.0; //tune
  const double kMaxTrackAgeNotTracking = .1;
  const double kMaxGoalTrackSmoothingTime = .5;
  const double kMaxGoalTrackAge = 2.5;
  const double kTrackStabilityWeight = 0.0;
  const double kTrackAgeWeight = 10.0;
  const double kTrackSwitchingWeight = 100.0;
  //ROBOT PHYSICAL CONSTANTS-need tuning: these set values are just to give an estimate
  const int kLongCANTimeoutMs = 100;
  const int kCANTimeoutMs = 10;
  const int kMotionProfileSlot = 0;
  const int kPositionPIDSlot = 1;
  const int kVelocityPIDSlot = 2;
  const int kMotionMagicPIDSlot = 3; //motion magic = smart motion;

  const double kBallDetectPassTime = .5;
  const double kBallDetectHoldTime = 1.0;


  const int kPCMID = 1;
  //Solenoid Ports
  const int kSolenoidID_WheelOfFortune = 0;
  const int kSolenoidID_Climber = 1;
  const int kSolenoidID_Intake = 2;
  const int kSolenoidID_DriveGear = 3;
  //DIO Ports
  const int kDIO_FirstBreak = 0;
  const int kDIO_FirstMake = 1;
  const int kDIO_LastBreak = 2;
  const int kDIO_LastMake = 3;
  const int kColor11_Blue = 4;
  const int kColor21_Green = 5;
  const int kColor12_Red = 6;
  const int kColor22_Yellow = 7;
  //AIO Ports

  //MIN MAX State Times
  const double kMaxPreExhausting = 30.0;
  const double kMinPreExhausting = 1.0;
  const double kMaxExhausting = 30.0;
  const double kMinExhausting = 0.0;
  const double kMaxIntaking = 150.0;
  const double kMinIntaking = 0.0;

  //Default Superstructure Speeds and Angles TODO: TUNE
  const double kBallPathBottomDefaultSpeed = .250;
  const double kBallPathTopDefaultSpeed = .250;
  const double kCenteringIntakeDefaultSpeed = .250;
  const double kShooterCloseDefaultSpeed = 100.0;
  const double kShooterMidDefaultSpeed = 200.0;
  const double kShooterFarDefaultSpeed = 300.0;
  const double kHoodDefaultAngle = 10.0;
  const double kBallPathBackTime = .25;
  const double kBallPathBackSpeed = -.25;
  const double kBallPathShootSpeed = .5;

  const double kSafetySausageInflatorDefault = .7; //percent out
  const double kWinchDefault = 25.0; //inches (Position PID)

  const double kWheelDefault = .5;

  const double kJoystickThreshold = 0.2;
  const int kSingleJoystickPort = 0;
  const int kSecondJoystickPort = 1;
  const double kJoystickHoldTime = .35;
  const bool kJoystickOne = true; //figure out how to set this
  //TODO: Tune
  const double kTurretJogMultiplier = 1.0;
  const double kTurretJogPower = 1.0;
  


  //Max drive Velocity and Acceleration
  const double kDriveMaxVelocity=75.0;//max recorded 88 inches/second
  const double kDriveMaxAcceleration=30.0;//max recorded 200 inches/second^2
  const double kDriveMaxCentripetalAcceleration=100.0;
  const double kDriveMaxVoltage=9.0;
  //Wheels
  const double kDriveWheelTrackWidthInches=27.5;
  const double kDriveWheelDiameterInches=6.25;//6.25;
  const double kDriveWheelRadiusInches=kDriveWheelDiameterInches/2.0;
  const double kTrackScrubFactor=1.1375;

  //Tuned dynamics...well not totally tuned yet (top 3 need to be)
  const double kRobotLinearInertia=60.0; //kg
  const double kRobotAngularInertia=10.0; //kg m^2
  const double kRobotAngularDrag=12.0; //N*m/(rad/sec)
  const double kDriveVIntercept=0.142557 ; // V
  const double kDriveKv=0.3944122*1.055; //V per rad/s
  const double kDriveKa =0.012; //V per rad/s^2 Ex: .012

  //Geometry
  const double kCenterToFrontBumperDistance= 38.25/2.0; //in
  const double kCenterToRearBumperDistance=38.25/2.0;
  const double kCenterToSideBumperDistance= 33.75/2.0;

  //Gearing and mechanical constexprants
  const double kDriveDownShiftVelocity= 5.75*12.0; //inches per second
  const double kDriveDownShiftAngularVelocity= kPI/2.0;
  const double kDriveUpShiftVelocity= 11.0*12.0; //inches per second


  //drive velocity loop (low Gear) TODO TUNE
  const double kDriveLowGearVelocityKp =0.0;//5e-5;
  const double kDriveLowGearVelocityKi = 0.0; //1e-6;
  const double kDriveLowGearVelocityKiZone = 0.0;
  const double kDriveLowGearVelocityKd = 0.0;
  const double kDriveLowGearVelocityKf =0.0;//.000015;
  const double kAcceleration = 200.0;

  //Drive Inches/Rotation
  const double kDriveHighGearIPR=100.0/38.67;//100.0/34.5;
  const double kDriveLowGearIPR=100.0/103.6;//100.0/93.5;
  const double kMotionPlannerLookAhead=.005;

}