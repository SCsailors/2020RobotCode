/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "frc/smartdashboard/SmartDashboard.h"

class Constants {
  double kPI= 3.14159265358979238463;
 public:
  //ROBOT PHYSICAL CONSTANTS-need tuning: these set values are just to give an estimate
  
  Constants();
  //Max drive Velocity and Acceleration
  const double kDriveMaxVelocity=75.0;//max recorded 88 inches/second
  const double kDriveMaxAcceleration=30.0;//max recorded 200 inches/second^2
  const double kDriveMaxCentripetalAcceleration=100.0;
  const double kDriveMaxVoltage=9.0;
  //Wheels
  const double kDriveWheelTrackWidthInches=27.5;
  const double kDriveWheelDiameterInches=6.25;//6.25;
  const double kDriveWheelRadiusInches=kDriveWheelDiameterInches/2.0;
  double kTrackScrubFactor=1.1375;

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

  //Gearing and mechanical constants
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
};
