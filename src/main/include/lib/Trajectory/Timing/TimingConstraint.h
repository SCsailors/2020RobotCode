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

#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Physics/DifferentialDrive.h"
#include "lib/Util/Units.h"

class TimingConstraint {
 public:
 double max_velocity=10.0;
  TimingConstraint(double max_centripetal_acceleration, shared_ptr<DifferentialDrive> drive, double abs_voltage_limit);

  class MinMaxAcceleration{
    public:
    double min_acceleration_=0.0;
    double max_acceleration_=0.0;
    //shared_ptr<MinMaxAcceleration> kNoLimits;

    MinMaxAcceleration();
    MinMaxAcceleration(double min_accel, double max_accel);
    double min_acceleration();
    double max_acceleration();
    bool valid();   
  };

  class CentripetalAccelerationConstraint{
    public:
    double max_centripetal_acceleration_=0.0;

    CentripetalAccelerationConstraint(double max_centripetal_acceleration);
    double getMaxVelocity(shared_ptr<Pose2DWithCurvature> state);
    shared_ptr<MinMaxAcceleration> getMinMaxAcceleration(shared_ptr<Pose2DWithCurvature> state, double velocity);
  };

  class DifferentialDriveDynamicConstraints{
    public:
    double max_velocity=9.0;
    shared_ptr<DifferentialDrive> drive_;
    double abs_voltage_limit_=0.0;
    DifferentialDriveDynamicConstraints(shared_ptr<DifferentialDrive> drive, double abs_voltage_limit);
    double getMaxVelocity(shared_ptr<Pose2DWithCurvature> state);
    shared_ptr<TimingConstraint::MinMaxAcceleration> getMinMaxAcceleration(shared_ptr<Pose2DWithCurvature> state, double velocity);
  };
  
  shared_ptr<TimingConstraint::CentripetalAccelerationConstraint> CAC;
  shared_ptr<TimingConstraint::DifferentialDriveDynamicConstraints> DDDC;
  shared_ptr<DifferentialDrive> drive;
  shared_ptr<TimingConstraint::MinMaxAcceleration> Min_Max_;
  double getMaxVelocity(shared_ptr<Pose2DWithCurvature> state);
  shared_ptr<TimingConstraint::MinMaxAcceleration> getMinMaxAcceleration(shared_ptr<Pose2DWithCurvature> state, double velocity);

};
