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

//if unit testing comment out line below
//#define FRC_ROBORIO true
//if using practice bot comment out line below
//#define COMPETITIONBOT true

#ifdef CompetitionBot
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "rev/CANPIDController.h"
#include "rev/CANSparkMaxLowLevel.h"
#include "AHRS.h"
#endif 


#include <memory>
#include <cmath>
#include <sstream>
#include <string>
#include <iomanip>
using namespace std;
namespace Subsystems{

class Drive : public Subsystems::Subsystem {
  public:
  Drive();
  
   enum DriveControlState{
    Open_Loop, //open loop voltage control
    Path_Following //PID/ nonlinear control
  };

  enum Shifterstate{
    Force_Low_Gear,
    Force_High_Gear,
    Auto_Shift,
    Manual
  }; 

  Shifterstate mShifterState = Shifterstate::Manual;
  bool mManualWantsHighGear = false;
  const double kShiftDelay = .1;

   double kP=0.013;//6E-7;//1E-7;//1E-4*.5;
   double kI=0.0;//1.864E-6*.35;
   double kD=0.14;//6E-6;//1.20713E-6*4;
   double kFF=0.00015;//0.00015;//0.0000015;//.000015;
   double kIZ=200.0;
   double kA=85.0;

  double prev_leftVel=0.0;
  double prev_rightVel=0.0;
   double p;
   double i;
   double d;
   double ff;
   double iz;
   double a;
   shared_ptr<DriveMotionPlanner::Output> output;
  
  Utility::TimeDelayedBoolean mAutoUpShift{};
  Utility::TimeDelayedBoolean mAutoDownShift{};

  class PeriodicIO{
    public:
    PeriodicIO();
    //inputs
    double left_rotations=0.0; //of Drive wheels
    double right_rotations=0.0;
    double left_distance=0.0;
    double right_distance=0.0;
    double left_velocity_rpm=0.0;
    double right_velocity_rpm=0.0;
    shared_ptr<Rotation2D> gyro_heading= make_shared<Rotation2D>();
    shared_ptr<Pose2D> error = make_shared<Pose2D>();

    //outputs
    double left_demand=0.0; //rotations per minute
    double right_demand=0.0;
    double left_accel=0.0;
    double right_accel=0.0;
    double left_feedforward=0.0;
    double right_feedforward=0.0;
    shared_ptr<TimedState> path_setpoint= make_shared<TimedState>();
  
  };
  
  //Hardware -
#ifdef CompetitionBot
  //AHRS NavX{SPI::Port::kMXP, 200}; //TODO check if this is the correct way to have a custom update rate.
  
  uint8_t UpdateRate=200;
  AHRS NavX{frc::SerialPort::kUSB1, AHRS::SerialDataType::kProcessedData, UpdateRate};
  rev::CANSparkMax leftMaster{10, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightMaster{12, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax leftSlave1{11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightSlave1{13, rev::CANSparkMax::MotorType::kBrushless};
  double mHighGearRatio= 2.668;
  double mGearingRatio=13.34;// TODO tune this
  rev::CANEncoder leftMasterEncoder=leftMaster.GetEncoder();
  rev::CANEncoder rightMasterEncoder=rightMaster.GetEncoder();
  
  rev::CANEncoder leftSlaveEncoder=leftSlave1.GetEncoder();
  rev::CANEncoder rightSlaveEncoder=rightSlave1.GetEncoder();
  //do we need more than one encoder per side?

  rev::CANPIDController leftMasterPID=leftMaster.GetPIDController();
  rev::CANPIDController rightMasterPID=rightMaster.GetPIDController();
  
  rev::CANPIDController leftSlavePID=leftSlave1.GetPIDController();
  rev::CANPIDController rightSlavePID=rightSlave1.GetPIDController();

  void configureSparkMaxPID();


//frc::DoubleSolenoid gearSolenoid{0,1};
frc::Solenoid mGearShifter{Constants::kPCMID, Constants::kSolenoidID_DriveGear};
#endif
  //PIDTuner.cpp sets it to true when it's run
  bool PIDTuning=false;
  
  //Control states
  DriveControlState mDriveControlState;

  //Hardware states
  shared_ptr<PeriodicIO> mPeriodicIO=make_shared<Drive::PeriodicIO>();
  bool mAutoShift=true;
  bool mIsHighGear;
  int prev_HighGear=0;
  bool mIsBrakeMode;
  shared_ptr<CSVWriter> mCSVWriter=NULL;
  shared_ptr<DriveMotionPlanner> mMotionPlanner;
  shared_ptr<Rotation2D> mGyroOffset= make_shared<Rotation2D>();
  bool mOverrideTrajectory = false;
  shared_ptr<DriveSignal> mDriveSignal=make_shared<DriveSignal>();
  
  double pi=3.14159265;

  int hGear=0;
  int lGear=0;
  
  void OnStart(double timestamp) override ;
  void OnLoop(double timestamp) override ;
  void OnStop(double timestamp) override ;

  double IPSToRadiansPerSecond(double inches_per_second);
  double rotationsToInches(double rotations);
  double RPMToInchesPerSecond(double rpm);
  double inchesToRotations(double inches);
  double inchesPerSecondToRPM(double inches_per_second);
  double radiansPerSecondToRPM(double rad_s);
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
  double getRightLinearvelocity();
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

  double leftAppliedOutput();
  double rightAppliedOutput();

  string getFields();
  string toCSV();
  template<typename T>
  string toString(T value);

  void setShifterState(Shifterstate state){mShifterState = state;}
  void setManualShifterState(bool wantsHighGear){mManualWantsHighGear = wantsHighGear;}

};
}
#include "Robot.h"

