/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include "lib/Splines/SplineGenerator.h"
#include "lib/Splines/QuinticHermiteSpline.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Trajectory/Timing/TimedState.h"

using namespace std;

class TrajectoryUtil {
 bool first_spline=true;
 double maxDx=2.0;
 double maxDy=.05;
 double maxDTheta=.1; 
 //same as in Spline Generator.h, don't actually influence anything
 
 shared_ptr<QuinticHermiteSpline> Quintic;
 
 
 public:
 vector<shared_ptr<QuinticHermiteSpline>> mQuintic;
 int i=0;
  TrajectoryUtil();
  vector<shared_ptr<Pose2DWithCurvature>> trajectoryFromSplineWaypoints(vector<shared_ptr<Pose2D>> waypoints, double maxDx, double maxDy, double maxDTheta);//creates a paramateterized list of Pose2DWithCurvature from Pose2Ds
  vector<shared_ptr<Pose2DWithCurvature>> trajectoryFromSplines(vector<shared_ptr<QuinticHermiteSpline>> splines, double maxDx, double maxDy, double maxDTheta);//creates Pose2DWC from list of Splines
  vector<shared_ptr<Pose2DWithCurvature>> trajectoryFromSplineWaypoints(vector<shared_ptr<Pose2D>> waypoints);
  vector<shared_ptr<TimedState>> mirrorTimed(vector<shared_ptr<TimedState>> trajectory);
};
