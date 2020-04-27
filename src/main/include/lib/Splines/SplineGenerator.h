/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Splines/QuinticHermiteSpline.h"
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Twist2D.h"
#include <vector>
#include <memory>
#include <iostream>
using namespace std;
 
class SplineGenerator {
  double kMaxDX=2.0;//inches
  double kMaxDY=.05;//inches
  double kMaxDTheta=.1;//radians
  double kMinSampleSize=1.0;
  double dt=1.0;
  
  int a=0;
  public:
  int i=0;
  int b=0;
  double ts0=0.0;
  double ts1=1.0;
  double t=0.0;
  std::vector<std::shared_ptr<Pose2DWithCurvature>> sample;
  std::vector<std::shared_ptr<Pose2DWithCurvature>> rv;
  shared_ptr<Translation2D> p0;
  shared_ptr<Translation2D> p1;
  
  shared_ptr<Rotation2D> r0;
  shared_ptr<Rotation2D> r1;
  vector<shared_ptr<Pose2DWithCurvature>> samples;


  SplineGenerator();
  std::vector<std::shared_ptr<Pose2DWithCurvature>> parameterizeSpline(shared_ptr<QuinticHermiteSpline> s, double maxDx, double maxDy, double maxDTheta, double at0, double at1);
  vector<shared_ptr<Pose2DWithCurvature>> parameterizeSplines(vector<shared_ptr<QuinticHermiteSpline>> splines, double maxDx, double maxDy, double maxDTheta);
  void getSegmentArc(shared_ptr<QuinticHermiteSpline> s, double t0, double t1, double maxDx, double maxDy, double maxDTheta);

 std::vector<std::shared_ptr<Pose2DWithCurvature>> parameterizeSpline(shared_ptr<QuinticHermiteSpline> s);
 vector<shared_ptr<Pose2DWithCurvature>> parameterizeSplines(vector<shared_ptr<QuinticHermiteSpline>> splines);

 void getSegmentArc(shared_ptr<QuinticHermiteSpline> s, double t0, double t1);
};
