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
#include "lib/Geometry/Twist2D.h"

#include <memory>
#include <cmath>
#include <vector>
#include <iostream>
using namespace std;

class Spline {
 public:
  Spline();
  virtual shared_ptr<Pose2D> getStartPose(){cout<<"in Spline"<<endl; return make_shared<Pose2D>();}
  virtual shared_ptr<Pose2D> getEndPose(){cout<<"in Spline"<<endl; return make_shared<Pose2D>();}

  virtual shared_ptr<Translation2D> getPoint(double t){cout<<"in Spline"<<endl; return make_shared<Translation2D>();}

  virtual double dx(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double ddx(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double dddx(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double dy(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double ddy(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double dddy(double t){cout<<"in Spline"<<endl; return 0.0;}

  virtual double getVelocity(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double getCurvature(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double getDCurvature(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual double dCurvature2(double t){cout<<"in Spline"<<endl; return 0.0;}
  virtual shared_ptr<Rotation2D> getHeading(double t){cout<<"in Spline"<<endl; return make_shared<Rotation2D>();}
  virtual shared_ptr<Pose2D> getPose2D(double t){cout<<"in Spline"<<endl; return make_shared<Pose2D>();}
  virtual shared_ptr<Pose2DWithCurvature> getPose2DWithCurvature(double t){cout<<"in Spline"<<endl; return make_shared<Pose2DWithCurvature>();}
  virtual double sumDCurvature2(){cout<<"in Spline"<<endl; return 0.0;}
};
