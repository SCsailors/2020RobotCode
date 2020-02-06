/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <Constants.h>

std::shared_ptr<Subsystems::LimelightConstants> Constants::kTurretLimelightConstants;
std::shared_ptr<Subsystems::LimelightConstants> Constants::kIntakeLimelightConstants;

std::shared_ptr<Subsystems::TalonConstants> Constants::kBallPathBottomConstants;
std::shared_ptr<Subsystems::TalonConstants> Constants::kBallPathTopConstants;
std::shared_ptr<Subsystems::TalonConstants> Constants::kCenteringIntakeConstants;
std::shared_ptr<Subsystems::TalonConstants> Constants::kHoodConstants;
std::shared_ptr<Subsystems::TalonConstants> Constants::kShooterConstants;
std::shared_ptr<Subsystems::SparkMaxConstants> Constants::kTurretConstants;


Constants::Constants()
{
    kTurretLimelightConstants = std::make_shared<Subsystems::LimelightConstants>("Turret Limelight", "limelight-turret", 0.0, std::make_shared<Pose2D>(0.0,0.0,0.0), std::make_shared<Rotation2D>(0.0,0.0, true));
    kIntakeLimelightConstants = std::make_shared<Subsystems::LimelightConstants>("Intake Limelight", "limelight-intake", 0.0, std::make_shared<Pose2D>(0.0,0.0,0.0), std::make_shared<Rotation2D>(0.0,0.0, true));
    
    //Servo Motor Subsystems constructors;
    kBallPathBottomConstants = std::make_shared<Subsystems::TalonConstants>();
    {

    }

    kBallPathTopConstants = std::make_shared<Subsystems::TalonConstants>();
    {
      
    }

    kCenteringIntakeConstants = std::make_shared<Subsystems::TalonConstants>();
    {

    }

    kHoodConstants = std::make_shared<Subsystems::TalonConstants>();
    {

    }
    
    kShooterConstants = std::make_shared<Subsystems::TalonConstants>();
    {

    }

    kTurretConstants = std::make_shared<Subsystems::SparkMaxConstants>();
    {

    }

     
  }