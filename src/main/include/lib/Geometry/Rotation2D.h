/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Geometry/Translation2D.h"

#include <memory>
#include <cmath>

using namespace std;

class Rotation2D {
  double magnitude;
  double kEpsilon=.000000001;
  double sin_angle_;
  double cos_angle_;

 public:
  Rotation2D();
  Rotation2D(double x, double y, bool normalize);
   double cos();
   double sin();
   shared_ptr<Rotation2D> fromRadians(double radians);
   shared_ptr<Rotation2D> fromDegrees(double degrees);
   double getRadians();
   double getDegrees();
  shared_ptr<Rotation2D> inverse();
  shared_ptr<Rotation2D> rotateBy(shared_ptr<Rotation2D> other);
  shared_ptr<Rotation2D> normal();
  shared_ptr<Rotation2D> interpolate(shared_ptr<Rotation2D> other, double x);
  bool isParallel(shared_ptr<Rotation2D> other);
  shared_ptr<Translation2D> toTranslation();
  
  double x_;
  double y_;
  double normalize_;
};
