/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <vector>
using namespace std;

#include <frc/commands/Subsystem.h>
#include <wpi/ArrayRef.h>

class WayPoints : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  vector<double> waypointsX{0.0};
  vector<double> waypointsY{0.0};
  vector<double> waypointsT{0.0};
  double prevnumberx=0.0;
  double currentnumberx=0.0;
  double prevnumbery=0.0;
  double currentnumbery=0.0;
  double prevtheta=0.0;
  double currenttheta=0.0;
  bool add=false;

 public:
  WayPoints();
  void InitDefaultCommand() override;
  void GetWayPoints();
  void PrintWayPoints();
  void InitializeWaypoints();
};
