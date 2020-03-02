/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


#include "Constants.h"

#include "Subsystems/Subsystem.h"
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Util/DriveSignal.h"
#include "lib/Util/CSVWriter.h"
#include "lib/Trajectory/TrajectoryIterator.h"
#include "Planners/DriveMotionPlanner.h"

#include "lib/Util/TimeDelayedBoolean.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>

#ifdef CompetitionBot
#include <ctre/Phoenix.h>
#include "AHRS.h"
#endif 

#include "Subsystems/ServoMotorSubsystem.h"

#include <memory>
#include <cmath>
#include <sstream>
#include <string>
#include <iomanip>

namespace Subsystems {

class FalconDrive : public Subsystems::Subsystem {
  static std::shared_ptr<FalconDrive> mInstance;
 public:
  FalconDrive(std::shared_ptr<TalonConstants> leftConstants, std::shared_ptr<TalonConstants> rightConstants);
  static std::shared_ptr<FalconDrive> getInstance();
  
  class PeriodicIO{
    public:
    PeriodicIO(){}
    //inputs
    int left_ticks = 0.0; //of Drive wheels
    int right_ticks = 0.0;
    double left_distance = 0.0;
    double right_distance = 0.0;
    int left_velocity_ticks = 0.0;
    int right_velocity_ticks = 0.0;
    double left_velocity_ips = 0.0;
    double right_velocity_ips = 0.0;
    std::shared_ptr<Rotation2D> gyro_heading = std::make_shared<Rotation2D>();
    std::shared_ptr<Pose2D> error = std::make_shared<Pose2D>();

    //outputs
    double left_demand=0.0;
    double right_demand=0.0;
    double left_accel=0.0;
    double right_accel=0.0;
    double left_feedforward=0.0;
    double right_feedforward=0.0;
    shared_ptr<TimedState> path_setpoint= make_shared<TimedState>();
  
};
  
  enum DriveControlState 
  {
    OPEN_LOOP,
    PATH_FOLLOWING
  };

  enum ShifterState
  {
    FORCE_LOW_GEAR,
    FORCE_HIGH_GEAR,
    AUTO_SHIFT
  };

  ShifterState mShifterState = ShifterState::FORCE_LOW_GEAR;

  std::shared_ptr<TalonConstants> mLeftConstants;
  std::shared_ptr<TalonConstants> mRightConstants;

  const double kShiftDelay = .1;
  double prev_leftVel=0.0;
  double prev_rightVel=0.0;
  double kP, kI, kD, kF, kIZ, kA;
  double mForwardSoftLimit = 0.0;
  double mReverseSoftLimit = 0.0;

  double DRIVE_ENCODER_PPR = 2048.0;
  
  std::shared_ptr<DriveMotionPlanner::Output> output;

  Utility::TimeDelayedBoolean mAutoUpShift{};
  Utility::TimeDelayedBoolean mAutoDownShift{};

    //Hardware -
#ifdef CompetitionBot
  AHRS NavX{frc::SerialPort::kUSB1, AHRS::SerialDataType::kProcessedData, 200};

  std::shared_ptr<TalonFX> mLeftMaster;
  std::shared_ptr<TalonFX> mRightMaster;

  std::vector<std::shared_ptr<TalonFX>> mLeftSlaves;
  std::vector<std::shared_ptr<TalonFX>> mRightSlaves;
  int i = 0;
  int j = 0;
  
  frc::Solenoid mGearShifter{Constants::kPCMID, Constants::kSolenoidID_DriveGear};
#endif

  //PIDTuner.cpp sets it to true when it's run
  bool PIDTuning = false;
  
  //Control states
  DriveControlState mDriveControlState;

  //Hardware states
  std::shared_ptr<FalconDrive::PeriodicIO> mPeriodicIO = std::make_shared<FalconDrive::PeriodicIO>();
  bool mAutoShift = true;
  bool mIsHighGear;
  int prev_HighGear = 0;
  bool mIsBrakeMode;
  std::shared_ptr<CSVWriter> mCSVWriter = NULL;
  std::shared_ptr<DriveMotionPlanner> mMotionPlanner;
  std::shared_ptr<Rotation2D> mGyroOffset = std::make_shared<Rotation2D>();
  bool mOverrideTrajectory = false;
  std::shared_ptr<DriveSignal> mDriveSignal = std::make_shared<DriveSignal>();

  void OnStart(double timestamp) override ;
  void OnLoop(double timestamp) override ;
  void OnStop(double timestamp) override ;

  //double IPSToRadiansPerSecond(double inches_per_second);
  double rotationsToInches(double rotations);
  double RPMToInchesPerSecond(double rpm);
  double inchesToRotations(double inches);
  double inchesPerSecondToRPM(double inches_per_second);
  double inchesPerSecondToTicksPer100ms(double inches_per_second);
  double radiansPerSecondToTicksPer100ms(double rad_s);
  void setOpenLoop(shared_ptr<DriveSignal> signal);
  void setVelocity(shared_ptr<DriveSignal> signal, shared_ptr<DriveSignal> feedforward);
  void setTrajectory(shared_ptr<TrajectoryIterator> trajectory);
  bool isDoneWithTrajectory();
  bool isHighGear();
  void setHighGear(bool wantsHighGear);
  bool isBrakeMode();
  void setBrakeMode(bool on);
  shared_ptr<Rotation2D> getHeading();
  void setHeading(shared_ptr<Rotation2D> heading);
  void stop();
  void zeroSensors();
  void resetEncoders();

  double getLeftEncoderRotations();
  double getRightEncoderRotations();
  double getRightEncoderDistance();
  double getLeftEncoderDistance();
  double getRightVelocityNativeUnits();
  double getLeftVelocityNativeUnits();
  double getRightRadsPerSec();
  double getLeftRadsPerSec();
  double getRightLinearVelocity();
  double getLeftLinearVelocity();
  double getLinearVelocity();
  double getAngularVelocity();
  void overrideTrajectory(bool value);
  void updatePathFollower();
  void handleAutoShift();
  void setPIDTuner(bool pidTuning);

  void writeToLog() override ;
  void readPeriodicInputs() override ;
  void writePeriodicOutputs() override ;
  bool checkSystem() override ;
  void outputTelemetry() override;
  
  void startLogging();
  void stopLogging();

  string getFields();
  string toCSV();
  template<typename T>
  string toString(T value);

  void setShifterState(ShifterState state){mShifterState = state;}  
};
}