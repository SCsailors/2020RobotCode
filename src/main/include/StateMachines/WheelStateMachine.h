/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <lib/Geometry/Pose2D.h>
#include <memory>
namespace StateMachines{
class WheelStateMachine {
 public:
  WheelStateMachine();
  
  void observeBall();

  void setTurretPosition(double position);
  void setTurretManualHeading(std::shared_ptr<Rotation2D> manualHeading);
  void jogTurret(double deltaP);
  
  double goToAutoScore();
  double goToAutoScore(double offsetOverride);

  void goToStowed();

  void goToScore();
  void goToPreScore();

  void goToPrepareForClimb();

  bool isInPreScoringPosition();
  bool isInScoringPosition();
};
}