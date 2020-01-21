/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <cmath>

#include <memory>
using namespace std;

class Rotation2D;

class Translation2D {

  

 public:
  Translation2D();
  Translation2D(double x, double y);
  Translation2D(shared_ptr<Translation2D> start, shared_ptr<Translation2D> end);
  double norm();
  double norm(double x, double y);
  double x();
  double y();
  shared_ptr<Translation2D> translateBy(shared_ptr<Translation2D> other);
  shared_ptr<Translation2D> rotateBy( shared_ptr<Rotation2D> rotation );

  shared_ptr<Translation2D> inverse();
  
  double distance(shared_ptr<Translation2D> other);
  shared_ptr<Translation2D> interpolate(shared_ptr<Translation2D> other, double x);
  shared_ptr<Translation2D> extrapolate(shared_ptr<Translation2D> other, double x);
  double cross(shared_ptr<Translation2D> a, shared_ptr<Translation2D> b);
  double x_=0.0;
  double y_=0.0;
  


};
