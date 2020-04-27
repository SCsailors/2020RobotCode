/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Twist2D.h"
#include <cmath>
#include <memory>
using namespace std;

class Pose2D {
  double kEps=.000001;
  double s;
  double c;
  double dtheta;
  double half_dtheta;
  double cos_minus_one;
  double halfdtheta_by_tan_halfdtheta;
  public:
  shared_ptr<Translation2D> translation_;
  shared_ptr<Rotation2D> rotation_;
   
 
  Pose2D();
  Pose2D(double x, double y, double degrees_theta); //theta is in degrees; 
  
  Pose2D(shared_ptr<Translation2D> translation, shared_ptr<Rotation2D> rotation);
  shared_ptr<Pose2D> exp(shared_ptr<Twist2D> delta);
  shared_ptr<Twist2D> log(shared_ptr<Pose2D> transform);
  shared_ptr<Pose2D>  interpolate(shared_ptr<Pose2D> other, double x);
  double distance(shared_ptr<Pose2D> other);
  shared_ptr<Pose2D> inverse();
  shared_ptr<Pose2D> normal();

  shared_ptr<Pose2D> transformBy(shared_ptr<Pose2D> other);

  shared_ptr<Translation2D> getTranslation();
  shared_ptr<Rotation2D> getRotation();
  shared_ptr<Pose2D> mirror();
  bool isColinear(shared_ptr<Pose2D> other);
  
  
};