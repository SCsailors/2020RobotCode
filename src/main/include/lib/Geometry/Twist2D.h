/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <memory>
#include <cmath>
using namespace std;

class Twist2D {
  

 public:
  double dx;
  double dy;
  double dtheta;
  Twist2D(double dx_, double dy_, double dtheta_);
  shared_ptr<Twist2D> scaled(double scale);
  double norm();
  shared_ptr<Twist2D> derive(shared_ptr<Twist2D> initial, double dt);
  Twist2D();
};
