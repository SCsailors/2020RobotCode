/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <vector>
#include "lib/Util/PolynomialRegression.h"
using namespace std;

class DriveCharacterization {
  int i=0;
 public:
 DriveCharacterization();
 
 class CharacterizationConstants{
    public:
    CharacterizationConstants();
    double ks=0.0; //voltage to break static friction
    double kv=0.0; //velocity
    double ka=0.0; //acceleration
    
  };
  
  
  class VelocityDataPoint{
    public:
      double velocity=0.0;
      double power=0.0;
      VelocityDataPoint(double v, double p);
  };

  class AccelerationDataPoint{
    public:
      double acceleration=0.0;
      double velocity=0.0;
      double power=0.0;
      AccelerationDataPoint(double v, double p, double a);
  };

  class CurvatureDataPoint{
    public:
      double linear_velocity=0.0;
      double angular_velocity=0.0;
      double left_voltage=0.0;
      double right_voltage=0.0;
      CurvatureDataPoint(double lv, double av, double lvol, double rvol);

  };
  vector<double> velocity;
  vector<double> power;
  vector<double> powerA;
  vector<double> acceleration;
  shared_ptr<CharacterizationConstants> characterizeDrive(vector<shared_ptr<VelocityDataPoint>> velocityData, vector<shared_ptr<AccelerationDataPoint>> accelerationData);
  shared_ptr<CharacterizationConstants> getVelocityCharacterization(vector<shared_ptr<VelocityDataPoint>> input);
  shared_ptr<CharacterizationConstants> getAccelerationCharacterization(vector<shared_ptr<AccelerationDataPoint>> input, shared_ptr<CharacterizationConstants> velocityConstants);
  shared_ptr<CharacterizationConstants> constants;

  
};
