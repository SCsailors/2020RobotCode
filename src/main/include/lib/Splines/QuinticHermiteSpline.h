/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Splines/Spline.h"


#include <memory>
#include <cmath>
#include <vector>
using namespace std;

class QuinticHermiteSpline: public Spline {
  double kEpsilon=1E-5;
  double kStepSize=1.0;
  double kMinDelta=.001;
  int kSamples=100;
  int kMaxIterations=100;
  double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;
  double Ax, Bx, Cx, Dx, Ex, Fx, Ay, By, Cy, Dy, Ey, Fy;
 public:
  QuinticHermiteSpline(shared_ptr<Pose2D> p0, shared_ptr<Pose2D> p1);
  QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1, 
                        double y0, double y1, double dy0, double dy1, double ddy0, double ddy1);
  void computeCoefficients();
  shared_ptr<Pose2D> getStartPose();
  shared_ptr<Pose2D> getEndPose() override;

  shared_ptr<Translation2D> getPoint(double t) override;

  double dx(double t) override;
  double ddx(double t) override;
  double dddx(double t) override;
  double dy(double t) override;
  double ddy(double t) override;
  double dddy(double t) override;

  double getVelocity(double t) override;
  double getCurvature(double t) override;
  double getDCurvature(double t) override;
  double dCurvature2(double t) override;
  shared_ptr<Rotation2D> getHeading(double t) override;
  shared_ptr<Pose2D> getPose2D(double t) override;
  shared_ptr<Pose2DWithCurvature> getPose2DWithCurvature(double t) override;
  double sumDCurvature2() override;
  double sumDCurvature2(vector<shared_ptr<QuinticHermiteSpline>> splines);
  class ControlPoint{
    public:
    ControlPoint(){}
    double ddx, ddy;
  };
  double optimizeSpline(vector<shared_ptr<QuinticHermiteSpline>> splines);
  void runOptimizationIteration(vector<shared_ptr<QuinticHermiteSpline>> splines);

  double fitParabola(shared_ptr<Translation2D> p1, shared_ptr<Translation2D> p2, shared_ptr<Translation2D> p3);
};
