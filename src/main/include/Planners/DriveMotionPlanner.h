/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <sstream>
#include <string>
#include <iomanip>

#include <cmath>
#include <vector>
#include <memory>

using namespace std;

#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Rotation2D.h"

#include "lib/Physics/DCMotorTransmission.h"
#include "lib/Physics/DifferentialDrive.h"

#include "lib/Trajectory/TrajectoryIterator.h"
#include "lib/Trajectory/TimedView.h"
#include "lib/Trajectory/TrajectorySamplePoint.h"
#include "lib/Trajectory/TrajectoryUtil.h"

#include "lib/Trajectory/Timing/ConstrainedState.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Trajectory/Timing/TimingConstraint.h"
#include "lib/Trajectory/Timing/TimingUtil.h"

#include "lib/Util/CSVWriter.h"
#include "lib/Util/Units.h"
#include "lib/Util/Util.h"


class DriveMotionPlanner {
  shared_ptr<CSVWriter> mCSVWriter;
  shared_ptr<Units> units=make_shared<Units>();
  shared_ptr<Util> util=make_shared<Util>();
  shared_ptr<TrajectoryUtil> trajUtil=make_shared<TrajectoryUtil>();
  shared_ptr<TimingUtil> timingUtil=make_shared<TimingUtil>();
  double kMaxDx=2.0;
  double kMaxDy=.25;
  double kMaxDTheta= units->toRadians(5.0);
 public:
  DriveMotionPlanner();

  class Output{
    shared_ptr<Units> unit=make_shared<Units>();

    
    shared_ptr<Util> util= make_shared<Util>();
    public:
    Output();
    Output(double leftVel, double rightVel, double leftAccel, double rightAccel, double leftVol, double rightVol);
    double left_velocity=0.0;
    double right_velocity=0.0;
    double left_acceleration=0.0;
    double right_acceleration=0.0;
    double left_feedforward_voltage=0.0;
    double right_feedforward_voltage=0.0;
    void flip();
};
    double mDt=0.0;
    bool PIDTuning=true;
    bool TrajectoryDone=true;
    shared_ptr<DriveMotionPlanner::Output> updatePID(shared_ptr<DifferentialDrive::DriveDynamics> dynamics, shared_ptr<Pose2D> current_state);
    shared_ptr<DriveMotionPlanner::Output> updateNonlinearFeedback(shared_ptr<DifferentialDrive::DriveDynamics> dynamics, shared_ptr<Pose2D> current_state);
    shared_ptr<DriveMotionPlanner::Output> update(double timestamp, shared_ptr<Pose2D> current_state);
    shared_ptr<DriveMotionPlanner::Output> updatePIDTuner(double startVelocity); //velocity of wheel encoders
    bool isDone();
    shared_ptr<Pose2D> error();
    shared_ptr<TimedState> setpoint();

  vector<shared_ptr<Pose2DWithCurvature>> flipped;
  enum FollowerType{FEEDFORWARD_ONLY, PID, NONLINEAR_FEEDBACK};
  FollowerType mFollowerType=NONLINEAR_FEEDBACK;//PID;//NONLINEAR_FEEDBACK;//FEEDFORWARD_ONLY;FEEDFORWARD_ONLY;
  void setFollowerType(FollowerType type);

  shared_ptr<TrajectoryIterator> mCurrentTrajectory;
  bool mIsReversed= false;
  double mLastTime= 1E100;
  shared_ptr<TimedState> mSetpoint= make_shared<TimedState>();
  shared_ptr<Pose2D> mError= make_shared<Pose2D>();
  shared_ptr<Output> mOutput= make_shared<Output>();
  shared_ptr<DifferentialDrive::ChassisState> prev_velocity_=make_shared<DifferentialDrive::ChassisState>();
  shared_ptr<DifferentialDrive> mModel;
  shared_ptr<TimingConstraint> constraints;
  shared_ptr<DifferentialDrive::ChassisState> adjusted_velocity;
  shared_ptr<DifferentialDrive::DriveDynamics> initialDynamics;
  shared_ptr<DifferentialDrive::DriveDynamics> adjustedDynamics=make_shared<DifferentialDrive::DriveDynamics>();
  shared_ptr<DifferentialDrive::DriveDynamics> dynamics;


  void setTrajectory(shared_ptr<TrajectoryIterator> trajectory);
  void reset();

  vector<shared_ptr<TimedState>> generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    shared_ptr<TimingConstraint> constraints,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage
  );

  vector<shared_ptr<TimedState>> generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    shared_ptr<TimingConstraint> constraints,
    double start_vel,
    double end_vel,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage
  );

  vector<shared_ptr<TimedState>> generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage
  );

  vector<shared_ptr<TimedState>> generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    double start_vel,
    double end_vel,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage
  );
  template<typename T>
  string toString(T value);
  string toCSV();
  string toPlannerCSV();
  string getFields();

  
};

