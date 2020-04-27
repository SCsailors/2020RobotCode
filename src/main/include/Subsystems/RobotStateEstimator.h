/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Subsystems/Subsystem.h"

#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Pose2D.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <memory>
using namespace std;

namespace Subsystems{
class RobotStateEstimator : public Subsystems::Subsystem{
 double left_encoder_prev_distance_=0.0;
 double right_encoder_prev_distance_=0.0;

 public:
  RobotStateEstimator();
  bool checkSystem() override ;
  void outputTelemetry() override ;
  void writeToLog() override ;
  void zeroSensors() override ;
  void readPeriodicInputs() override ;
  void writePeriodicOutputs() override ;
  void OnStart(double timestamp) override ;
  void OnLoop(double timestamp) override ;
  void OnStop(double timestamp) override ;
};
}
//has to be below otherwise the static variable in robot will be an undefined type--it took two hours to figure that out.
#include "Robot.h"
