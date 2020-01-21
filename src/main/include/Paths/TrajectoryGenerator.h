/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "lib/Trajectory/Timing/TimingConstraint.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Physics/DifferentialDrive.h"
#include "lib/Physics/DCMotorTransmission.h"
#include "lib/Trajectory/TrajectoryUtil.h"


#include "frc/smartdashboard/SmartDashboard.h"

#include <memory>
#include <vector>
using namespace std;

class TrajectoryGenerator {

 public:
  TrajectoryGenerator();
  
  class TrajectorySet{
    public:
    double kMaxVelocity= 130.0; // only example data, tuned data is in constants file
    double kMaxAccel= 130.0;
    double kMaxCentripetalAccel=100.0;
    double kMaxVoltage = 9.0;
    
    TrajectorySet();
    class MirroredTrajectory{
      public:
      MirroredTrajectory(vector<shared_ptr<TimedState>> right);
      vector<shared_ptr<TimedState>> get(bool left);
      vector<shared_ptr<TimedState>> mLeft;
      vector<shared_ptr<TimedState>> mRight;
      shared_ptr<TrajectoryUtil> trajectoryUtil;
    };
    //Critical Poses go here
    //Competition Poses
    
    //Test Poses here
    shared_ptr<Pose2D> origin = make_shared<Pose2D>();
    shared_ptr<Pose2D> forwardStraightEnd = make_shared<Pose2D>(170.0, 0.0, 0.0);

    shared_ptr<Pose2D> forwardSwerveEnd= make_shared<Pose2D>(100.0, -30.0, 0.0);

    shared_ptr<Pose2D> forwardSwerveAngledEnd = make_shared<Pose2D>(100.0, -30.0, 40.0);
    

    
    //Origin is in the center of the middle alliance wall, 
    //+y is left, -y is right, 
    //+x is towards the center of the field
    //defined for case that Robot starts on the right, then mirrored about +x axis for left side
    
    //Competition

    //Test
    shared_ptr<TrajectorySet::MirroredTrajectory> DriveForwardStraightTest;
    shared_ptr<TrajectorySet::MirroredTrajectory> DriveForwardSwerveTest;
    shared_ptr<TrajectorySet::MirroredTrajectory> DriveForwardSwerveAngledTest;

    shared_ptr<TrajectorySet::MirroredTrajectory> DriveReverseStraightTest;
    shared_ptr<TrajectorySet::MirroredTrajectory> DriveReverseSwerveTest;
    shared_ptr<TrajectorySet::MirroredTrajectory> DriveReverseSwerveAngledTest;
    
    //get Trajectories go here
    //Competition

    //Test
    vector<shared_ptr<TimedState>> getDriveForwardStraightTest();
    vector<shared_ptr<TimedState>> getDriveForwardSwerveTest();
    vector<shared_ptr<TimedState>> getDriveForwardSwerveAngledTest();
    
    vector<shared_ptr<TimedState>> getDriveReverseStraightTest();
    vector<shared_ptr<TimedState>> getDriveReverseSwerveTest();
    vector<shared_ptr<TimedState>> getDriveReverseSwerveAngledTest();
    
  
  vector<shared_ptr<TimedState>> generateTrajectories(
    bool reversed,
    vector<shared_ptr<Pose2D>> waypoints,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage);

  vector<shared_ptr<TimedState>> generateTrajectories(
    bool reversed,
    vector<shared_ptr<Pose2D>> waypoints,
    double start_vel, //inches/s
    double end_vel, //inches/s
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage);
  };
  shared_ptr<TrajectorySet> Trajectories;
  void generateTrajectories();
  shared_ptr<TrajectorySet> getTrajectorySet(){
    return Trajectories;
  }
};
#include "Planners/DriveMotionPlanner.h"
#include "Robot.h"