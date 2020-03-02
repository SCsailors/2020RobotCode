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
#include "lib/Util/MovingAverageTwist2D.h"

#include "lib/Vision/GoalTracker.h"
#include "lib/Vision/AimingParameters.h"
#include "lib/Vision/TargetInfo.h"

#include "Subsystems/Limelight.h"

#include <opencv2/opencv.hpp>

#include "Kinematics.h"

#include <sstream>
#include <string>
#include <iomanip>
#include <memory>
using namespace std;

namespace FRC_7054{

class RobotState {
 /*RobotState keeps track of various coordinate frames throughout the match. 
  *A coordinate frame is simply a point and direction in space that defines an (x,y) coordinate system. 
  *Transforms keep track of the spatial relationship between different frames.
  *
  * Robot frames of interest (from parent to child)
  * 
  * 1. Field frame: origin is where the robot is turned on.
  * 
  * 2. Vehicle frame: origin is the center of the robot wheelbase, facing forwards.
  * 
  * 3. Turret frame: origin is the center of the turret, which is coincident with the origin of the vehicle frame, but with potentially different angle.
  * 
  * 4. Camera frame: origin is the center of the Limelight imager relative to the turret.
  * 
  * 5. Goal frame: origin is the center of the vision target, facing outwards along the normal: can be multiple
  * 
  * As a kinematic chain with 5 frames, there are 4 transforms of interest
  * 
  * 1. Field-to-Vehicle: tracked over time by entegrating encoder and gyro measurements. Drifts, but accurate over short periods.
  * 
  * 2. Vehicle-to-Turret: constant transform and variable rotation meausred by turret encoder.
  * 
  * 3. Turret-to-Camera: Constant
  * 
  * 4. Camera-to-goal: Measured by vision system.
  */
  shared_ptr<Pose2D> pose=make_shared<Pose2D>();
  int kObservationBufferSize=100;
  static std::shared_ptr<RobotState> mInstance;
 public:
 shared_ptr<InterpolatingTreeMap> field_to_vehicle = make_shared<InterpolatingTreeMap>(kObservationBufferSize);
 shared_ptr<InterpolatingTreeMap> vehicle_to_turret = make_shared<InterpolatingTreeMap>(kObservationBufferSize);
 shared_ptr<Twist2D> vehicle_velocity_predicted_=make_shared<Twist2D>();
 shared_ptr<Twist2D> vehicle_velocity_measured_=make_shared<Twist2D>();
 Utility::MovingAverageTwist2D vehicle_velocity_measured_filtered_{25};



 double distance_driven_=0.0;

 VisionTargeting::GoalTracker vision_target_turret{};
 VisionTargeting::GoalTracker vision_target_intake{};

 std::vector<std::shared_ptr<Pose2D>> mCameraToVisionTargetPosesTurret;
 std::vector<std::shared_ptr<Pose2D>> mCameraToVisionTargetPosesIntake; 

  const std::vector<std::shared_ptr<Pose2D>> emptyVector{};
  
  RobotState();
  
  static std::shared_ptr<RobotState> getInstance();
  
  void reset(double start_time, shared_ptr<Pose2D> initial_field_to_vehicle, shared_ptr<Pose2D> initial_vehicle_to_turret);
  void reset(double start_time, shared_ptr<Pose2D> initial_field_to_vehicle);
  void reset();
  
  shared_ptr<Pose2D> getFieldToVehicle(double timestamp);
  shared_ptr<Pose2D> getVehicleToTurret(double timestamp);
  shared_ptr<Pose2D> getFieldToTurret(double timestamp);
  
  shared_ptr<Pose2D> getLatestFieldToVehicle();
  shared_ptr<Pose2D> getLatestVehicleToTurret();
  shared_ptr<Pose2D> getPredictedFieldToVehicle(double lookahead_time);
  
  void addFieldToVehicleObservation(double timestamp, shared_ptr<Pose2D> observation);
  void addVehicleToTurretObservation(double timestamp, shared_ptr<Pose2D> observation);
  
  void addObservation(double timestamp, shared_ptr<Twist2D> measured_velocity, shared_ptr<Twist2D> predicted_velocity);
  shared_ptr<Twist2D> generateOdometryFromSensors(double left_encoder_delta_distance, double right_encoder_delta_distance, shared_ptr<Rotation2D> current_gyro_angle);
  
  double getDistanceDriven();
  void resetDistanceDriven();
  shared_ptr<Twist2D> getPredictedVelocity();
  shared_ptr<Twist2D> getMeasuredVelocity();
  shared_ptr<Twist2D> getSmoothedVelocity();
  
  void resetVision();
  std::shared_ptr<Pose2D> getCameraToVisionTargetPose(VisionTargeting::TargetInfo target, bool turret, std::shared_ptr<Subsystems::Limelight> source);
  void updateGoalTracker(double timestamp, std::vector<std::shared_ptr<Pose2D>> goalPose, VisionTargeting::GoalTracker goalTracker, std::shared_ptr<Subsystems::Limelight> source);
  void addVisionUpdate(double timestamp, std::vector<VisionTargeting::TargetInfo> observations, std::vector<std::shared_ptr<Subsystems::Limelight>> limelights);


  std::vector<double> kPossibleNormals{0.0, 180.0};
  shared_ptr<Pose2D> getFieldToVisionTarget(bool turret);
  shared_ptr<Pose2D> getVehicleToVisionTarget(double timestamp, bool turret);

  VisionTargeting::AimingParameters getAimingParameters(bool turret, int prev_track_id, double max_track_age);

  void outputToSmartDashboard();
  string toPlannerCSV();

  string toString(double value);
};
}
