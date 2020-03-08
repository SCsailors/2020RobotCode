/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/FalconDrive.h"

#include "lib/Drivers/TalonFactory.h"
#include <cmath>
#include "Constants.h"

#include <RobotState.h>

using namespace Subsystems;
std::shared_ptr<FalconDrive> FalconDrive::mInstance;

FalconDrive::FalconDrive(std::shared_ptr<TalonConstants> leftConstants, std::shared_ptr<TalonConstants> rightConstants) 
{
    #ifdef CompetitionBot
    mLeftConstants = leftConstants;
    mRightConstants = rightConstants;

    mLeftMaster = Drivers::TalonFactory::createDefaultTalonFX(mLeftConstants->id);
    mRightMaster = Drivers::TalonFactory::createDefaultTalonFX(mRightConstants->id);

    //Left Drive
    mForwardSoftLimit = ((mLeftConstants->kMaxUnitsLimit - mLeftConstants->kHomePosition)*mLeftConstants->kTicksPerUnitDistance);    
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigForwardSoftLimitThreshold((int) mForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure forward soft limit: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigForwardSoftLimitEnable(mLeftConstants->kEnableForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable forward soft limit: ");

    mReverseSoftLimit = ((mLeftConstants->kMinUnitsLimit - mLeftConstants->kHomePosition)*mLeftConstants->kTicksPerUnitDistance);
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigReverseSoftLimitThreshold((int) mReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure reverse soft limit: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigReverseSoftLimitEnable(mLeftConstants->kEnableReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable reverse soft limit: ");

    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigVoltageCompSaturation(mLeftConstants->kVoltageCompensation, Constants::kLongCANTimeoutMs), ": could not configure Voltage Compensation: ");

    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kP(Constants::kMotionProfileSlot, mLeftConstants->kP.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kP: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kI(Constants::kMotionProfileSlot, mLeftConstants->kI.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kI: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kD(Constants::kMotionProfileSlot, mLeftConstants->kD.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kD: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kF(Constants::kMotionProfileSlot, mLeftConstants->kF.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kF: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_IntegralZone(Constants::kMotionProfileSlot, mLeftConstants->kIZone.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigMaxIntegralAccumulator(Constants::kMotionProfileSlot, mLeftConstants->kMaxIAccum.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigAllowableClosedloopError(Constants::kMotionProfileSlot, mLeftConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kP(Constants::kPositionPIDSlot, mLeftConstants->kP.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kI(Constants::kPositionPIDSlot, mLeftConstants->kI.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kD(Constants::kPositionPIDSlot, mLeftConstants->kD.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kF(Constants::kPositionPIDSlot, mLeftConstants->kF.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_IntegralZone(Constants::kPositionPIDSlot, mLeftConstants->kIZone.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigMaxIntegralAccumulator(Constants::kPositionPIDSlot, mLeftConstants->kMaxIAccum.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigAllowableClosedloopError(Constants::kPositionPIDSlot, mLeftConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kP(Constants::kVelocityPIDSlot, mLeftConstants->kP.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kI(Constants::kVelocityPIDSlot, mLeftConstants->kI.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kD(Constants::kVelocityPIDSlot, mLeftConstants->kD.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_kF(Constants::kVelocityPIDSlot, mLeftConstants->kF.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->Config_IntegralZone(Constants::kVelocityPIDSlot, mLeftConstants->kIZone.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigMaxIntegralAccumulator(Constants::kVelocityPIDSlot, mLeftConstants->kMaxIAccum.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigAllowableClosedloopError(Constants::kVelocityPIDSlot, mLeftConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot Deadband: ");

    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigMotionCruiseVelocity(mLeftConstants->kMaxVelocity, Constants::kLongCANTimeoutMs), ": could not configure cruise velocity: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigMotionAcceleration(mLeftConstants->kMaxAcceleration, Constants::kLongCANTimeoutMs), ": could not configure acceleration: ");
    
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigClosedloopRamp(mLeftConstants->kClosedLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure closed loop ramp rate: ");
    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigOpenloopRamp(mLeftConstants->kOpenLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure open loop ramp rate: ");

    Drivers::TalonFactory::handleCANError(mLeftConstants->id, mLeftMaster->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, (double) mLeftConstants->kPeakCurrentLimit, (double) mLeftConstants->kContinuousCurrentLimit, (double) mLeftConstants->kPeakCurrentDuration}, Constants::kLongCANTimeoutMs), ": could not configure Supply Current Limit: ");
    
    mLeftMaster->EnableVoltageCompensation(mLeftConstants->kEnableVoltageCompensation);
    mLeftMaster->SetInverted(mLeftConstants->inverted);
    mLeftMaster->SetSensorPhase(mLeftConstants->kInvertSensorPhase);
    mLeftMaster->SetNeutralMode(mLeftConstants->kNeutralMode);
    mLeftMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
    mLeftMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
    mLeftMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mLeftConstants->kStatusFrame8UpdateRate, 20);

    if (mLeftConstants->kSlaveIDs.size() == 0)
    {
        std::cout << "TalonFXSubsystem: Skipping Slaves" << std::endl;   
    } else
    {
        
    for (auto slave : mLeftConstants->kSlaveIDs)
    {
        std::cout<< "Creating Slave TalonFX: " << slave->id <<slave->invert_motor << slave->isTalonSRX << std::endl;
        if ((slave->id) == -1)
        {
            std::cout << "Not Creating TalonFX Slave: " << slave->id << "for :" << mLeftConstants->id << std::endl;
            continue;
        }
        mLeftSlaves.push_back(Drivers::TalonFactory::createSlaveTalonFX(slave->id, mLeftMaster));
        
        mLeftSlaves.at(i)->SetInverted(slave->invert_motor);
        mLeftSlaves.at(i)->SetNeutralMode(mLeftConstants->kNeutralMode);
        i++;
        
    }    
    }
    //Right Drive
    mForwardSoftLimit = ((mRightConstants->kMaxUnitsLimit - mRightConstants->kHomePosition)*mRightConstants->kTicksPerUnitDistance);    
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigForwardSoftLimitThreshold((int) mForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure forward soft limit: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigForwardSoftLimitEnable(mRightConstants->kEnableForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable forward soft limit: ");

    mReverseSoftLimit = ((mRightConstants->kMinUnitsLimit - mRightConstants->kHomePosition)*mRightConstants->kTicksPerUnitDistance);
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigReverseSoftLimitThreshold((int) mReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure reverse soft limit: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigReverseSoftLimitEnable(mRightConstants->kEnableReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable reverse soft limit: ");

    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigVoltageCompSaturation(mRightConstants->kVoltageCompensation, Constants::kLongCANTimeoutMs), ": could not configure Voltage Compensation: ");

    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kP(Constants::kMotionProfileSlot, mRightConstants->kP.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kP: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kI(Constants::kMotionProfileSlot, mRightConstants->kI.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kI: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kD(Constants::kMotionProfileSlot, mRightConstants->kD.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kD: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kF(Constants::kMotionProfileSlot, mRightConstants->kF.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kF: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_IntegralZone(Constants::kMotionProfileSlot, mRightConstants->kIZone.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigMaxIntegralAccumulator(Constants::kMotionProfileSlot, mRightConstants->kMaxIAccum.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigAllowableClosedloopError(Constants::kMotionProfileSlot, mRightConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kP(Constants::kPositionPIDSlot, mRightConstants->kP.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kI(Constants::kPositionPIDSlot, mRightConstants->kI.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kD(Constants::kPositionPIDSlot, mRightConstants->kD.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kF(Constants::kPositionPIDSlot, mRightConstants->kF.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_IntegralZone(Constants::kPositionPIDSlot, mRightConstants->kIZone.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigMaxIntegralAccumulator(Constants::kPositionPIDSlot, mRightConstants->kMaxIAccum.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigAllowableClosedloopError(Constants::kPositionPIDSlot, mRightConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kP(Constants::kVelocityPIDSlot, mRightConstants->kP.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kI(Constants::kVelocityPIDSlot, mRightConstants->kI.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kD(Constants::kVelocityPIDSlot, mRightConstants->kD.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_kF(Constants::kVelocityPIDSlot, mRightConstants->kF.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->Config_IntegralZone(Constants::kVelocityPIDSlot, mRightConstants->kIZone.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigMaxIntegralAccumulator(Constants::kVelocityPIDSlot, mRightConstants->kMaxIAccum.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigAllowableClosedloopError(Constants::kVelocityPIDSlot, mRightConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot Deadband: ");

    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigMotionCruiseVelocity(mRightConstants->kMaxVelocity, Constants::kLongCANTimeoutMs), ": could not configure cruise velocity: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigMotionAcceleration(mRightConstants->kMaxAcceleration, Constants::kLongCANTimeoutMs), ": could not configure acceleration: ");
    
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigClosedloopRamp(mRightConstants->kClosedLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure closed loop ramp rate: ");
    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigOpenloopRamp(mRightConstants->kOpenLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure open loop ramp rate: ");

    Drivers::TalonFactory::handleCANError(mRightConstants->id, mRightMaster->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, (double) mRightConstants->kPeakCurrentLimit, (double) mRightConstants->kContinuousCurrentLimit, (double) mRightConstants->kPeakCurrentDuration}, Constants::kLongCANTimeoutMs), ": could not configure Supply Current Limit: ");
    
    mRightMaster->EnableVoltageCompensation(mRightConstants->kEnableVoltageCompensation);
    mRightMaster->SetInverted(mRightConstants->inverted);
    mRightMaster->SetSensorPhase(mRightConstants->kInvertSensorPhase);
    mRightMaster->SetNeutralMode(mRightConstants->kNeutralMode);
    mRightMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
    mRightMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
    mRightMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mRightConstants->kStatusFrame8UpdateRate, 20);

    if (mRightConstants->kSlaveIDs.size() == 0)
    {
        std::cout << "TalonFXSubsystem: Skipping Slaves" << std::endl;   
    } else
    {
        
    for (auto slave : mRightConstants->kSlaveIDs)
    {
        std::cout<< "Creating Slave TalonFX: " << slave->id <<slave->invert_motor << slave->isTalonSRX << std::endl;
        if ((slave->id) == -1)
        {
            std::cout << "Not Creating TalonFX Slave: " << slave->id << "for :" << mRightConstants->id << std::endl;
            continue;
        }
        
        mRightSlaves.push_back(Drivers::TalonFactory::createSlaveTalonFX(slave->id, mRightMaster));
        
        mRightSlaves.at(j)->SetInverted(slave->invert_motor);
        mRightSlaves.at(j)->SetNeutralMode(mRightConstants->kNeutralMode);
        j++;
        
    }    
    }
    #endif

    mIsHighGear=true;
    setHighGear(false);

    setOpenLoop(mDriveSignal->NEUTRAL);

    mIsBrakeMode=true;
    setBrakeMode(false);

    mMotionPlanner=make_shared<DriveMotionPlanner>();
}

std::shared_ptr<FalconDrive> FalconDrive::getInstance()
{
    if (!mInstance)
    {
        std::shared_ptr<TalonConstants> leftConstants = std::make_shared<TalonConstants>();
        leftConstants->id = 10;
        leftConstants->kName = "Left Drive Master";
        leftConstants->kIsTalonSRX = false;
        leftConstants->kClosedLoopRampRate = Constants::kDriveOpenRampRateLowGear;
        leftConstants->kOpenLoopRampRate = Constants::kDriveOpenRampRateLowGear;
        leftConstants->kNeutralMode  = NeutralMode::Coast;
        leftConstants->kStatusFrame8UpdateRate = 50;
        leftConstants->kVoltageCompensation = 12.5;
        leftConstants->kEnableVoltageCompensation = true;
        leftConstants->kP.at(2) = .9;
        leftConstants->kD.at(2) = 10.0;
        
        std::shared_ptr<TalonConstants> rightConstants = std::make_shared<TalonConstants>();
        rightConstants->id = 11;
        rightConstants->kName = "Right Drive Master";
        rightConstants->kIsTalonSRX = false;
        rightConstants->inverted = true;
        rightConstants->kClosedLoopRampRate = Constants::kDriveOpenRampRateLowGear;
        rightConstants->kOpenLoopRampRate = Constants::kDriveOpenRampRateLowGear;
        rightConstants->kNeutralMode  = NeutralMode::Coast;
        rightConstants->kStatusFrame8UpdateRate = 50;
        rightConstants->kVoltageCompensation = 12.5;
        rightConstants->kEnableVoltageCompensation = true;
        rightConstants->kP.at(2) = .9;
        rightConstants->kD.at(2) = 10.0;
        mInstance = std::make_shared<FalconDrive>(leftConstants, rightConstants);
    }
    return mInstance;
}

void FalconDrive::OnStart(double timestamp)
{
    setBrakeMode(false);
}

void FalconDrive::OnLoop(double timestamp)
{
    if (mDriveControlState == DriveControlState::OPEN_LOOP)
    {

    } else if (mDriveControlState == DriveControlState::PATH_FOLLOWING)
    {
        if (!PIDTuning)
        {
            updatePathFollower();
        }
    }

    frc::SmartDashboard::PutBoolean("AutoShift Value", mAutoShift);

    switch (mShifterState)
    {
    case FORCE_LOW_GEAR:
        setHighGear(false);
        break;
    case FORCE_HIGH_GEAR:
        setHighGear(true);
        break;
    case AUTO_SHIFT:
        handleAutoShift();
        break;
    }
}

void FalconDrive::OnStop(double timestamp)
{
    stop();
}

//Add Conversions
double FalconDrive::rotationsToInches(double rotations)
{
    return rotations * (mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR);
}

double FalconDrive::RPMToInchesPerSecond(double rpm)
{
    return rotationsToInches(rpm) / 60.0;
}

double FalconDrive::inchesToRotations(double inches)
{
    return inches / (mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR);
}

double FalconDrive::inchesPerSecondToRPM(double inches_per_second)
{
    return inchesToRotations(inches_per_second) * 60.0;
}

double FalconDrive::inchesPerSecondToTicksPer100ms(double inches_per_second)
{
    return inchesToRotations(inches_per_second) / 10.0 * DRIVE_ENCODER_PPR;
}

double FalconDrive::radiansPerSecondToTicksPer100ms(double rad_s)
{
    return inchesPerSecondToTicksPer100ms(rad_s * Constants::kDriveWheelRadiusInches);
}

void FalconDrive::setOpenLoop(std::shared_ptr<DriveSignal> signal)
{
    #ifdef CompetitionBot
    if (mDriveControlState != OPEN_LOOP)
    {
        setBrakeMode(false);
        mAutoShift = true;
        mLeftMaster->ConfigNeutralDeadband(0.04, 0);
        mRightMaster->ConfigNeutralDeadband(0.04, 0);
        mDriveControlState = OPEN_LOOP;
    }

    mPeriodicIO->left_demand = signal->getLeft();
    mPeriodicIO->right_demand = signal->getRight();

    frc::SmartDashboard::PutNumber("DRIVE / LEFT Demand", mPeriodicIO->left_demand);
    frc::SmartDashboard::PutNumber("DRIVE / RIGHT Demand", mPeriodicIO->right_demand);
    frc::SmartDashboard::PutNumber("DRIVE / ControlMode", mDriveControlState);

    mPeriodicIO->left_feedforward = 0.0;
    mPeriodicIO->right_feedforward = 0.0;
    #endif
}

void FalconDrive::setVelocity(std::shared_ptr<DriveSignal> signal, std::shared_ptr<DriveSignal> feedforward)
{
    #ifdef CompetitionBot
    if(mDriveControlState!= PATH_FOLLOWING){
        //entering velocity controlled state
        setBrakeMode(true);
        mAutoShift=false;
        mLeftMaster->SelectProfileSlot(Constants::kVelocityPIDSlot, 0);
        mRightMaster->SelectProfileSlot(Constants::kVelocityPIDSlot, 0);

        mLeftMaster->ConfigNeutralDeadband(0.0, 0);
        mRightMaster->ConfigNeutralDeadband(0.0, 0);
        mDriveControlState= PATH_FOLLOWING;
    }

    mPeriodicIO->left_demand= signal->getLeft();
    mPeriodicIO->right_demand= signal->getRight();
    mPeriodicIO->left_feedforward=feedforward->getLeft();
    mPeriodicIO->right_feedforward=feedforward->getRight();
    #endif
}

void FalconDrive::setTrajectory(std::shared_ptr<TrajectoryIterator> trajectory)
{
    mOverrideTrajectory = false;
    mMotionPlanner->reset();
    mMotionPlanner->setTrajectory(trajectory);
    mDriveControlState = PATH_FOLLOWING;
}

bool FalconDrive::isDoneWithTrajectory()
{
    if (mDriveControlState != PATH_FOLLOWING)
    {
        return false;
    } 
    return mMotionPlanner->isDone() || mOverrideTrajectory;
}

bool FalconDrive::isHighGear()
{
    return mIsHighGear;
}

void FalconDrive::setHighGear(bool wantsHighGear)
{
    if (wantsHighGear != mIsHighGear)
    {
        if (wantsHighGear)
        {
            mIsHighGear = true;
            #ifdef CompetitionBot
            mGearShifter.Set(true);
            mLeftMaster->ConfigOpenloopRamp(Constants::kDriveOpenRampRateHighGear);
            mRightMaster->ConfigOpenloopRamp(Constants::kDriveOpenRampRateHighGear);
            #endif
        } else if (!wantsHighGear)
        {
            mIsHighGear = false;
            #ifdef CompetitionBot
            mGearShifter.Set(false);
            mLeftMaster->ConfigOpenloopRamp(Constants::kDriveOpenRampRateLowGear);
            mRightMaster->ConfigOpenloopRamp(Constants::kDriveOpenRampRateLowGear);
            #endif
        }
    }
    frc::SmartDashboard::PutString("Drive/ Setting Drive Gear?", mIsHighGear? "HighGear": "LowGear"); 
}

bool FalconDrive::isBrakeMode()
{
    return mIsBrakeMode;
}

void FalconDrive::setBrakeMode(bool on)
{
    if (mIsBrakeMode != on)
    {
        mIsBrakeMode = on;

        if (mIsBrakeMode)
        {
            #ifdef CompetitionBot
            mLeftMaster->SetNeutralMode(NeutralMode::Brake);
            mRightMaster->SetNeutralMode(NeutralMode::Brake);

            for (auto slave: mLeftSlaves)
            {
                slave->SetNeutralMode(NeutralMode::Brake);
            }

            for (auto slave: mRightSlaves)
            {
                slave->SetNeutralMode(NeutralMode::Brake);
            }
            #endif
        } else
        {
            #ifdef CompetitionBot
            mLeftMaster->SetNeutralMode(NeutralMode::Coast);
            mRightMaster->SetNeutralMode(NeutralMode::Coast);

            for (auto slave: mLeftSlaves)
            {
                slave->SetNeutralMode(NeutralMode::Coast);
            }

            for (auto slave: mRightSlaves)
            {
                slave->SetNeutralMode(NeutralMode::Coast);
            }
            #endif
        }
        
    }
}

std::shared_ptr<Rotation2D> FalconDrive::getHeading()
{
    return mPeriodicIO->gyro_heading;
}

void FalconDrive::setHeading(std::shared_ptr<Rotation2D> heading)
{
    mPeriodicIO->gyro_heading = heading;
    #ifdef CompetitionBot
    mGyroOffset = Rotation2D::fromDegrees(0.0);//mGyroOffset = heading->rotateBy(Rotation2D::fromDegrees(NavX.GetFusedHeading())->inverse());
    #endif
}

void FalconDrive::stop()
{
    setOpenLoop(mDriveSignal->NEUTRAL);
}

void FalconDrive::outputTelemetry()
{
    frc::SmartDashboard::PutNumber("Drive/Right Encoder Distance", mPeriodicIO->right_distance);//check first four
    frc::SmartDashboard::PutNumber("Drive/Left Encoder Distance", mPeriodicIO->left_distance);
    frc::SmartDashboard::PutNumber("Drive/Right Encoder Rotations", mPeriodicIO->right_ticks);
    frc::SmartDashboard::PutNumber("Drive/Left Encoder Rotations", mPeriodicIO->left_ticks);
    frc::SmartDashboard::PutNumber("Drive/Right Linear Velocity", getRightLinearVelocity());
    frc::SmartDashboard::PutNumber("Drive/Left Linear Velocity", getLeftLinearVelocity());

    frc::SmartDashboard::PutNumber("Drive/X Error", mPeriodicIO->error->inverse()->getTranslation()->x());
    frc::SmartDashboard::PutNumber("Drive/Y Error", mPeriodicIO->error->inverse()->getTranslation()->y());
    frc::SmartDashboard::PutNumber("Drive/Theta Error", mPeriodicIO->error->inverse()->getRotation()->getDegrees());

    frc::SmartDashboard::PutBoolean("Drive/ Is High Gear", isHighGear());

    if (getHeading()   != NULL){
        frc::SmartDashboard::PutNumber("Drive/Gyro Heading", getHeading()->getDegrees());
    }
}

void FalconDrive::zeroSensors()
{
    mPeriodicIO = std::make_shared<FalconDrive::PeriodicIO>();
    std::shared_ptr<Rotation2D> rotation = std::make_shared<Rotation2D>();
    setHeading(rotation);
    resetEncoders();
    mAutoShift = true;
}

void FalconDrive::resetEncoders()
{
    mPeriodicIO = std::make_shared<FalconDrive::PeriodicIO>();
    #ifdef CompetitionBot
    mLeftMaster->SetSelectedSensorPosition(0);
    mRightMaster->SetSelectedSensorPosition(0);
    #endif
}

//Add Conversions
double FalconDrive::getLeftEncoderRotations()
{ //of motors
    return mPeriodicIO->left_ticks / DRIVE_ENCODER_PPR;
} 

double FalconDrive::getRightEncoderRotations()
{
    return mPeriodicIO->right_ticks / DRIVE_ENCODER_PPR;
}

double FalconDrive::getLeftEncoderDistance()
{ //distance in inches of wheels
    getLeftEncoderRotations() * (mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR);
}

double FalconDrive::getRightEncoderDistance()
{
    getRightEncoderRotations() * (mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR);
}

double FalconDrive::getLeftVelocityNativeUnits()
{
    return mPeriodicIO->left_velocity_ticks;
}

double FalconDrive::getRightVelocityNativeUnits()
{
    return mPeriodicIO->right_velocity_ticks;
}

double FalconDrive::getLeftRadsPerSec()
{
    getLeftLinearVelocity() / Constants::kDriveWheelRadiusInches;
}

double FalconDrive::getRightRadsPerSec()
{
    getRightLinearVelocity() / Constants::kDriveWheelRadiusInches;
}

double FalconDrive::getLeftLinearVelocity()
{
    return getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR * (mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR);
}

double FalconDrive::getRightLinearVelocity()
{
    return getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR * (mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR);
}

double FalconDrive::getLinearVelocity()
{
    return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
}

double FalconDrive::getAngularVelocity()
{
    return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants::kDriveWheelTrackWidthInches;
}

void FalconDrive::overrideTrajectory(bool value)
{
    mOverrideTrajectory = value;
}

void FalconDrive::updatePathFollower()
{
    if (mDriveControlState == PATH_FOLLOWING)
    {
        double now = frc::Timer::GetFPGATimestamp();
        std::shared_ptr<Pose2D> pose = FRC_7054::RobotState::getInstance()->getFieldToVehicle(now);

        output = mMotionPlanner->update(now, pose);

        mPeriodicIO->error = mMotionPlanner->mError;

        std::shared_ptr<DriveSignal> signal = std::make_shared<DriveSignal>(radiansPerSecondToTicksPer100ms(output->left_velocity), radiansPerSecondToTicksPer100ms(output->right_velocity));
        std::shared_ptr<DriveSignal> feedforward = std::make_shared<DriveSignal>(output->left_feedforward_voltage / 12.0, output->right_feedforward_voltage / 12.0);

        if (!mOverrideTrajectory)
        {
            setVelocity(signal, feedforward);
            mPeriodicIO->left_accel = radiansPerSecondToTicksPer100ms(output->left_acceleration) / 1000.0;
            mPeriodicIO->right_accel = radiansPerSecondToTicksPer100ms(output->right_acceleration) / 1000.0;
        } else
        {
            setVelocity(signal->BRAKE, signal->BRAKE);
            mPeriodicIO->left_accel = mPeriodicIO->right_accel = 0.0;
        }
        
    }
}

void FalconDrive::handleAutoShift()
{
    double linear_velocity = std::fabs(getLinearVelocity());
    double angular_velocity = std::fabs(getAngularVelocity());

    if (mAutoDownShift.update(mIsHighGear && linear_velocity < Constants::kDriveDownShiftVelocity, kShiftDelay))
    {
        setHighGear(false);
    } else if (mAutoUpShift.update(!mIsHighGear && linear_velocity > Constants::kDriveDownShiftVelocity, kShiftDelay))
    {
        setHighGear(true);
    }
}

void FalconDrive::setPIDTuner(bool pidTuning)
{
    PIDTuning = pidTuning;
}

void FalconDrive::writeToLog()
{

}

void FalconDrive::readPeriodicInputs()
{
    double prevLeftTicks = mPeriodicIO->left_ticks;
    double prevRightTicks = mPeriodicIO->right_ticks;
    #ifdef CompetitionBot
    mPeriodicIO->left_ticks = mLeftMaster->GetSelectedSensorPosition(0);
    mPeriodicIO->right_ticks = mRightMaster->GetSelectedSensorPosition(0);
    mPeriodicIO->left_velocity_ticks = mLeftMaster->GetSelectedSensorVelocity(0);
    mPeriodicIO->right_velocity_ticks = mRightMaster->GetSelectedSensorVelocity(0);
    mPeriodicIO->gyro_heading = Rotation2D::fromDegrees(0.0);//Rotation2D::fromDegrees(NavX.GetFusedHeading())->rotateBy(mGyroOffset);

    double deltaLeftInches = rotationsToInches((mPeriodicIO->left_ticks - prevLeftTicks) / DRIVE_ENCODER_PPR);
    mPeriodicIO->left_distance += deltaLeftInches;

    double deltaRightInches = rotationsToInches((mPeriodicIO->right_ticks - prevRightTicks) / DRIVE_ENCODER_PPR);
    mPeriodicIO->right_distance += deltaRightInches;

    if (mCSVWriter != NULL)
    {
        mCSVWriter->add(toCSV());
    }
    #endif
}

void FalconDrive::writePeriodicOutputs()
{
    #ifdef CompetitionBot
    if (mDriveControlState == DriveControlState::OPEN_LOOP)
    {
        
        mLeftMaster->Set(ControlMode::PercentOutput, mPeriodicIO->left_demand, DemandType::DemandType_ArbitraryFeedForward, 0.0);
        mRightMaster->Set(ControlMode::PercentOutput, mPeriodicIO->right_demand, DemandType::DemandType_ArbitraryFeedForward, 0.0);
    } else
    {
        mLeftMaster->Set(ControlMode::Velocity, mPeriodicIO->left_demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO->left_feedforward + Constants::kDriveLowGearVelocityKd * mPeriodicIO->left_accel / 1023.0);
        mRightMaster->Set(ControlMode::Velocity, mPeriodicIO->right_demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO->right_feedforward + Constants::kDriveLowGearVelocityKd * mPeriodicIO->right_accel / 1023.0);
    }
    #endif
}

bool FalconDrive::checkSystem()
{
    return true;
}

void FalconDrive::startLogging()
{
    if(mCSVWriter==NULL){
        mCSVWriter=make_shared<CSVWriter>("Drive-Logs", getFields());    
    }
}

void FalconDrive::stopLogging()
{
    if(mCSVWriter!=NULL){
        mCSVWriter->close();
        mCSVWriter=NULL;
    }
}

template<typename T>
string FalconDrive::toString(T value)
{
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}

string FalconDrive::getFields()
{
    return "T, left_rotations, right_rotations,left_distance,right_distance,left_velocity_rpm,right_velocity_rpm,left_demand,right_demand,left_accel,right_accel,left_feedforward,right_feedforward, X, Y, Theta, Linear Velocity, Left Applied Output, Right Applied Output, Left Output Error, Right Output Error,";
}

string FalconDrive::toCSV()
{
    shared_ptr<Pose2D> pose = FRC_7054::RobotState::getInstance()->getLatestFieldToVehicle();
    return toString(frc::Timer::GetFPGATimestamp())+","+toString(mPeriodicIO->left_ticks)+","+toString(mPeriodicIO->left_ticks)+","+toString(mPeriodicIO->left_distance)+","+toString(mPeriodicIO->right_distance)+","+toString(mPeriodicIO->left_velocity_ticks)+","+toString(mPeriodicIO->right_velocity_ticks)+","+
    toString(mPeriodicIO->left_demand)+","+toString(mPeriodicIO->right_demand)+","+toString(mPeriodicIO->left_accel)+","+toString(mPeriodicIO->right_accel)+","+toString(mPeriodicIO->left_feedforward)+","+toString(mPeriodicIO->right_feedforward)+","+toString(pose->getTranslation()->x())+","+toString(pose->getTranslation()->y())+","+toString(pose->getRotation()->getDegrees())+","+toString(getLinearVelocity())
    #ifdef CompetitionBot
    //+","+toString(mLeftMaster.GetAppliedOutput())+","+toString(rightMaster.GetAppliedOutput())+","+toString(mPeriodicIO->left_demand-getLeftVelocityNativeUnits())+","+toString(mPeriodicIO->right_demand-getRightVelocityNativeUnits())
    #endif
    ;
}