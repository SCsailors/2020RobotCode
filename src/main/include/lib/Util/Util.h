/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <cmath>
#include <sstream>
#include <string>
#include <iomanip>
using namespace std;

class Util {
 public:
 double pi= 3.14159265358979238463;
 double kEpsilon=1E-4;
  Util();
  double limit(double v, double min, double max);
  double limit(double v, double lim);
  double interpolate(double a, double b, double x);
  bool epsilonEquals(double a, double b);
  bool epsilonEquals(double a, double b, double epsilon);
  string toString(double value);
};
