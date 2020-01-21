/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Util/Util.h"
#include <vector>
#include <memory>
#include <cmath>
using namespace std;

class QuinticSpline {
  shared_ptr<Util> util= make_shared<Util>();
  double t; //0-1 (percentage of spline)
  double x0, x1, dx0, dx1, ddx0, ddx1; //parameters
  double y0, y1, dy0, dy1, ddy0, ddy1;
  double scale;

 public:
  QuinticSpline();
  QuinticSpline(shared_ptr<Pose2D> p0, shared_ptr<Pose2D> p1, bool first);
  std::vector<QuinticSpline> splines;

  double h0(double t);//basis functions
  double h1(double t);
  double h2(double t);
  double h3(double t);
  double h4(double t);
  double h5(double t);
  
  double dh0(double t);//first derivatives
  double dh1(double t);
  double dh2(double t);
  double dh3(double t);
  double dh4(double t);
  double dh5(double t);

  double ddh0(double t);//second derivatives
  double ddh1(double t);
  double ddh2(double t);
  double ddh3(double t);
  double ddh4(double t);
  double ddh5(double t);

  double dddh0(double t);//third derivatives
  double dddh1(double t);
  double dddh2(double t);
  double dddh3(double t);
  double dddh4(double t);
  double dddh5(double t);

  shared_ptr<Pose2D> getStartPose();
  shared_ptr<Pose2D> getEndPose();

  shared_ptr<Translation2D> getPoint(double t);

  double dx(double t);
  double ddx(double t);
  double dddx(double t);
  double dy(double t);
  double ddy(double t);
  double dddy(double t);

  double getVelocity(double t);
  double getCurvature(double t);
  double getDCurvature(double t);
  shared_ptr<Rotation2D> getHeading(double t);
  shared_ptr<Pose2D> getPose2D(double t);
  shared_ptr<Pose2DWithCurvature> getPose2DWithCurvature(double t);
};
