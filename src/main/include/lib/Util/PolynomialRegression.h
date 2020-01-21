/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <cmath>
using namespace std;

class PolynomialRegression {
  int n=0;
 public:
  vector<double> d;
  
  PolynomialRegression();
  PolynomialRegression( vector<double> x, vector<double> y, int degree);
  void Solve(vector<double> x, vector<double> y, int degree);
  double beta(int j);
  int degree();

  double predict(double x);
};
