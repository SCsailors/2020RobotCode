/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/smartdashboard/SmartDashboard.h"

#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Twist2D.h"
#include "lib/Util/InterpolatingTreeMap.h"
#include "Kinematics.h"

#include <sstream>
#include <string>
#include <iomanip>
#include <memory>
using namespace std;



class RobotState {
  shared_ptr<Pose2D> pose=make_shared<Pose2D>();
  int kObservationBufferSize=100;
 public:
 shared_ptr<InterpolatingTreeMap> field_to_vehicle;
 shared_ptr<Twist2D> vehicle_velocity_predicted_=make_shared<Twist2D>();
 shared_ptr<Twist2D> vehicle_velocity_measured_=make_shared<Twist2D>();
 double distance_driven_=0.0;
  
  RobotState();
  void reset(double start_time, shared_ptr<Pose2D> initial_field_to_vehicle);
  void resetDistanceDriven();
  
  shared_ptr<Pose2D> getFieldToVehicle(double timestamp);
  shared_ptr<Pose2D> getLatestFieldToVehicle();
  shared_ptr<Pose2D> getPredictedFieldToVehicle(double lookahead_time);
  void addFieldToVehicleObservation(double timestamp, shared_ptr<Pose2D> observation);
  void addObservation(double timestamp, shared_ptr<Twist2D> measured_velocity, shared_ptr<Twist2D> predicted_velocity);
  shared_ptr<Twist2D> generateOdometryFromSensors(double left_encoder_delta_distance, double right_encoder_delta_distance, shared_ptr<Rotation2D> current_gyro_angle);
  
  double getDistanceDriven();
  shared_ptr<Twist2D> getPredictedVelocity();
  shared_ptr<Twist2D> getMeasuredVelocity();
  
  void outputToSmartDashboard();
  string toPlannerCSV();

  string toString(double value);
};

//has to be below otherwise the static variable in robot will be an undefined type--it took two hours to figure that out.
#include "Robot.h"