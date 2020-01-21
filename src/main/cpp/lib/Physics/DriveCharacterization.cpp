/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Physics/DriveCharacterization.h"

DriveCharacterization::DriveCharacterization() {}

DriveCharacterization::CharacterizationConstants::CharacterizationConstants(){} 

DriveCharacterization::VelocityDataPoint::VelocityDataPoint(double v, double p){
    this->velocity=v;
    this->power=p;
}

DriveCharacterization::AccelerationDataPoint::AccelerationDataPoint(double v, double p, double a){
    velocity=v;
    power=p;
    acceleration=a;
}

DriveCharacterization::CurvatureDataPoint::CurvatureDataPoint(double lv, double av, double lvol, double rvol){
    this->linear_velocity=lv;
    this->angular_velocity=av;
    this->left_voltage=lvol;
    this->right_voltage=rvol;
}

shared_ptr<DriveCharacterization::CharacterizationConstants> DriveCharacterization::characterizeDrive(vector<shared_ptr<VelocityDataPoint>> velocityData, vector<shared_ptr<AccelerationDataPoint>> accelerationData){
  shared_ptr<CharacterizationConstants> rv=getVelocityCharacterization(velocityData);
  getAccelerationCharacterization(accelerationData, rv);
  return rv;
}

shared_ptr<DriveCharacterization::CharacterizationConstants> DriveCharacterization::getVelocityCharacterization(vector<shared_ptr<VelocityDataPoint>> input){
    shared_ptr<CharacterizationConstants> constants= make_shared<CharacterizationConstants>();
    for(shared_ptr<VelocityDataPoint> data: input){
        if(data->velocity>1E-4){
            velocity.push_back(data->velocity);
            power.push_back(data->power);
        }
    }
    shared_ptr<PolynomialRegression> p= make_shared<PolynomialRegression>(velocity, power, 1);
    constants->ks= p->beta(0);
    constants->kv=p->beta(1);
    return constants;
}

shared_ptr<DriveCharacterization::CharacterizationConstants> DriveCharacterization::getAccelerationCharacterization(vector<shared_ptr<AccelerationDataPoint>> input, shared_ptr<CharacterizationConstants> velocityConstants){
    for(shared_ptr<AccelerationDataPoint> data: input){
        acceleration.push_back(data->acceleration);
        powerA.push_back(data->power-velocityConstants->kv*input.at(i)->velocity-velocityConstants->ks);
    }
    shared_ptr<PolynomialRegression> p=make_shared<PolynomialRegression>(acceleration, powerA, 1);
    velocityConstants->ka=p->beta(1);
   return velocityConstants;
}