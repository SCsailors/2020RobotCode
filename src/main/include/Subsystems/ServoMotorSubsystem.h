/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Subsystems/Subsystem.h>

#include <rev/CANSparkMax.h>
#include <rev/CANDigitalInput.h>
#include <ctre/Phoenix.h>

#include <lib/Drivers/TalonFactory.h>
#include <lib/Drivers/SparkMaxFactory.h>
#include <lib/Util/Util.h>

#include <string>
#include <vector>
#include <memory>

#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace Subsystems {
class SlaveConstants {
  public:
    SlaveConstants(){}
    SlaveConstants(int id, bool invert_motor, bool isTalonSRX) : id(id), invert_motor(invert_motor), isTalonSRX(isTalonSRX) {}
    int id = -1;
    bool invert_motor = false;
    bool isTalonSRX = false;
};

class ServoMotorSubsystemConstants {
 public:
  ServoMotorSubsystemConstants(){}
  std::vector<std::shared_ptr<SlaveConstants>> kSlaveIDs;
  int id = -1;
  std::string kName = "ERROR_ASSIGN_A_NAME";
  double kHomePosition = 0.0; //Units
  double kTicksPerUnitDistance = 1.0;
  bool inverted = false;
  bool invert_sensor_phase = false;
  double kMinUnitsLimit = -std::numeric_limits<double>::max();
  double kMaxUnitsLimit = std::numeric_limits<double>::max();
  bool readInputs = true;
  

  //see Constants.h for which slot is for which purpose 
  std::vector<double> kP{0.0, 0.0, 0.0, 0.0};
  std::vector<double> kI{0.0, 0.0, 0.0, 0.0};
  std::vector<double> kD{0.0, 0.0, 0.0, 0.0};
  std::vector<double> kF{0.0, 0.0, 0.0, 0.0};
  std::vector<double> kIZone{0.0, 0.0, 0.0, 0.0};
  std::vector<double> kMaxIAccum{0.0, 0.0, 0.0, 0.0};

  //Smart/Magic motion
  double kAllowableClosedLoopError = 0.0;
  double kMaxVelocity = 0.0;
  double kMaxAcceleration = 0.0;
  
  double kClosedLoopRampRate = 0.0;
  double kOpenLoopRampRate = 0.0;

  bool kEnableVoltageCompensation = true;
  double kVoltageCompensation = 12.5;

  bool kEnableForwardSoftLimit = false;
  double kForwardSoftLimit = 0.0;

  bool kEnableReverseSoftLimit = false;
  double kReverseSoftLimit = 0.0;

  bool kRecoverPositionOnReset = false;
};

class TalonConstants : public ServoMotorSubsystemConstants {
 public:
  TalonConstants(){}
  bool kInvertSensorPhase = false;
  bool kIsTalonSRX = false;
  int kContinuousCurrentLimit = 20; //amps
  int kPeakCurrentLimit = 60; //amps
  int kPeakCurrentDuration = 200; //milliseconds
  //TalonFX uses these slightly differentyly to configure the SupplyCurrentLimitConfig
  NeutralMode kNeutralMode = NeutralMode::Brake;
  int kStatusFrame8UpdateRate = 1000;
};

class SparkMaxConstants : public ServoMotorSubsystemConstants {
 public:
  SparkMaxConstants(){}
  std::vector<double> kDFilter{0.0, 0.0, 0.0, 0.0};
  rev::ControlType kControlType = rev::ControlType::kDutyCycle;
  rev::CANSparkMax::MotorType kMotorType = rev::CANSparkMax::MotorType::kBrushless;
  rev::CANSparkMax::IdleMode kIdleMode = rev::CANSparkMax::IdleMode::kBrake;

  rev::CANDigitalInput::LimitSwitchPolarity kForwardLimitSwitchPolarity{rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen};
  bool kEnableForwardLimitSwitch = false;
  rev::CANDigitalInput::LimitSwitchPolarity kReverseLimitSwitchPolarity{rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen};
  bool kEnableReverseLimitSwitch = false;

  rev::CANEncoder::EncoderType kEncoderType = rev::CANEncoder::EncoderType::kHallSensor; 
  rev::CANEncoder::AlternateEncoderType kAltEncoderType = rev::CANEncoder::AlternateEncoderType::kQuadrature;
  bool kIsAltEncoder = false;
  bool kIsAbsEncoder = false;
  //add analog sensor as alternate feedback sensor
  //check to see if the encoders have to have their units converted
  int kCountsPerRev = 42.0;

  int kSecondaryCurrentLimit = 100;
  int kCurrentStallLimit = 80;
  int kCurrentFreeLimit = 20;

};

class PeriodicIO {
    public:
      PeriodicIO(){}
      //INPUTS
      double timestamp = 0.0;
      int position_ticks= 0;
      double position_units = 0.0;
      int velocity_ticks_per_100ms = 0;
      double velocity_units = 0.0; //Units/second
      int active_trajectory_position = 0;
      int active_trajectory_velocity = 0;
      int active_trajectory_acceleration = 0;
      double output_percent = 0.0;
      double output_voltage = 0.0;
      double master_current = 0.0;
      int error_ticks = 0;
      int encoder_wraps = 0;
      int absolute_offset = 0;
      int absolute_position = 0;
      int absolute_position_modded = 0;
      bool reset_occurred = false;

      //OUTPUTS
      double demand = 0.0;
      double feedforward = 0.0;
  };

class ServoMotorSubsystem : public Subsystems::Subsystem {
 public:
  ServoMotorSubsystem();
  int getAbsoluteEncoderRawPosition(int pulseWidthPosition, int CPR);
  double getPositionTicks();
  double getPosition();
  double getVelocity();
  virtual void handleMasterReset(bool reset){}
  double mForwardSoftLimit; 
  double mReverseSoftLimit;
  enum ControlState{MOTION_PROFILING, POSITION_PID, VELOCITY_PID, MOTION_MAGIC, OPEN_LOOP};
  ControlState mControlState = ControlState::OPEN_LOOP;
  std::shared_ptr<PeriodicIO> mPeriodicIO = std::make_shared<PeriodicIO>();
  bool mHasBeenZeroed = false;
  Util util{};

  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double kf = 0.0;
  double demand = 0.0;
  double feedforward = 0.0;
  bool PIDTuning = false;
  

  ControlState mPIDMode = ControlState::OPEN_LOOP;
  frc::SendableChooser<ControlState> mModeChooser;
};

class SparkMaxSubsystem : public ServoMotorSubsystem {
    double prev_position_units = NAN;
    double prev_timestamp = 0.0;
  public:
    SparkMaxSubsystem(std::shared_ptr<SparkMaxConstants> constants);
    SparkMaxSubsystem() : ServoMotorSubsystem() {}
    void readPeriodicInputs() override;
    void writePeriodicOutputs() override;
    void zeroSensors() override;
    void OnStart(double timestamp) override;
    void OnLoop(double timestamp) override;
    void OnStop(double timestamp) override;
    void outputTelemetry() override;


    bool hasFinishedTrajectory();
    double getActiveTrajectoryUnits();
    double getActiveTrajectoryUnitsPerSecond();
    double getPredictedPositionUnits(double lookahead_secs);
    bool atHomingLocation();
    void resetIfAtLimit();
    bool hasBeenZeroed();
    void stop();
    int estimateSensorPositionFromAbsolute();

    double getSetpoint();
    void setSetpointMotionMagic(double units, double feedforward_v);
    void setSetpointMotionMagic(double units);
    void setSetpointPositionPID(double units, double feedforward_v);
    void setSetpointVelocityPID(double units, double feedforward_v);
    void setGoalMotionProfiling(){}
    void setOpenLoop(double percentage);
    
    double ticksToUnits(double ticks);
    double ticksToHomedUnits(double ticks);
    double unitsToTicks(double units);
    double homeAwareUnitsToTicks(double units);
    double constrainTicks(double ticks);
    double ticksPer100msToUnitsPerSecond(double ticks_per_100ms);
    double unitsPerSecondToTicksPer100ms(double units_per_second);
    void pidTuning();

    std::shared_ptr<SparkMaxConstants> mConstants;
    std::shared_ptr<rev::CANSparkMax> mMaster;
    std::vector<std::shared_ptr<rev::CANSparkMax>> mSlaves;
    std::shared_ptr<rev::CANEncoder> mEncoder = NULL;
    std::shared_ptr<rev::CANDigitalInput> mForwardLimitSwitch = NULL;
    std::shared_ptr<rev::CANDigitalInput> mReverseLimitSwitch = NULL;
    std::shared_ptr<rev::CANPIDController> mPIDController = NULL;
    int j = 0;
    int k = 0;
};


class TalonSubsystem : public ServoMotorSubsystem {
    
  public:
    

    
    TalonSubsystem() : ServoMotorSubsystem() {};

    void outputTelemetry() override;

    bool hasFinishedTrajectory();
    double getActiveTrajectoryUnits();
    double getActiveTrajectoryUnitsPerSecond();
    double getPredictedPositionUnits(double lookahead_secs);
    bool atHomingLocation();
    void resetIfAtLimit();
    int getAbsoluteEncoderRawPosition(int pulse_width_position);
    bool hasBeenZeroed();
    void stop();
    int estimateSensorPositionFromAbsolute();
    virtual void handleMasterReset(bool reset){};


    double getSetpoint();
    void setSetpointMotionMagic(double units, double feedforward_v);
    void setSetpointMotionMagic(double units);
    void setSetpointPositionPID(double units, double feedforward_v);
    void setSetpointVelocityPID(double units, double feedforward_v);
    void setGoalMotionProfiling(){}
    void setOpenLoop(double percentage);
    
    double ticksToUnits(double ticks);
    double ticksToHomedUnits(double ticks);
    double unitsToTicks(double units);
    double homeAwareUnitsToTicks(double units);
    double constrainTicks(double ticks);
    double ticksPer100msToUnitsPerSecond(double ticks_per_100ms);
    double unitsPerSecondToTicksPer100ms(double units_per_second);
    
    std::shared_ptr<TalonConstants> mConstants;
    
    StickyFaults mFaults{};
};

class TalonSRXSubsystem : public TalonSubsystem {
  public:
    TalonSRXSubsystem(std::shared_ptr<TalonConstants> constants);
    void readPeriodicInputs() override;
    void writePeriodicOutputs() override;
    void zeroSensors() override;

    
    void OnStart(double timestamp) override;
    void OnLoop(double timestamp) override;
    void OnStop(double timestamp) override;
    void pidTuning();
    
    std::shared_ptr<TalonSRX> mMaster;
    std::vector<std::shared_ptr<BaseTalon>> mSlaves;
    int j = 0;
    int k = 0;
};


class TalonFXSubsystem : public TalonSubsystem {
  public:
    TalonFXSubsystem(std::shared_ptr<TalonConstants> constants);
    void readPeriodicInputs() override;
    void writePeriodicOutputs() override;
    void zeroSensors() override;

    void OnStart(double timestamp) override;
    void OnLoop(double timestamp) override;
    void OnStop(double timestamp) override;
    
    std::shared_ptr<TalonFX> mMaster;
    std::vector<std::shared_ptr<BaseTalon>> mSlaves;
    int j = 0;
    int k = 0;

    void pidTuning();
    
    
};

}