/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/ServoMotorSubsystem.h"
#include <Constants.h>
using namespace Subsystems;
ServoMotorSubsystem::ServoMotorSubsystem(){}

int ServoMotorSubsystem::getAbsoluteEncoderRawPosition(int pulseWidthPosition, int CPR)
{
    int abs_raw_with_rollover = pulseWidthPosition % CPR;
    return abs_raw_with_rollover + (abs_raw_with_rollover < 0 ? abs_raw_with_rollover + CPR : 0);
}

//ticks
double ServoMotorSubsystem::getPositionTicks()
{
    return (double) mPeriodicIO.position_ticks;
}

//units
double ServoMotorSubsystem::getPosition()
{
    return mPeriodicIO.position_units;
}

//units/sec
double ServoMotorSubsystem::getVelocity()
{
    return mPeriodicIO.velocity_units;
}

SparkMaxSubsystem::SparkMaxSubsystem(SparkMaxConstants constants)
{
    mConstants = constants;
    mMaster = Drivers::SparkMaxFactory::createDefaultSparkMax(mConstants.id, mConstants.kMotorType);

    mPIDController = std::make_shared<rev::CANPIDController>(mMaster->GetPIDController());

    if (mConstants.kEnableForwardLimitSwitch)
    {
        mForwardLimitSwitch = std::make_shared<rev::CANDigitalInput>(*mMaster.get(), rev::CANDigitalInput::LimitSwitch::kForward, mConstants.kForwardLimitSwitchPolarity);
    }
    
    if (mConstants.kEnableReverseLimitSwitch)
    {
        mReverseLimitSwitch = std::make_shared<rev::CANDigitalInput>(*mMaster.get(), rev::CANDigitalInput::LimitSwitch::kReverse, mConstants.kReverseLimitSwitchPolarity);
    }

    if (mConstants.kIsAltEncoder)
    {
        mEncoder = std::make_shared<rev::CANEncoder>(*mMaster.get(), mConstants.kAltEncoderType, mConstants.kCountsPerRev);
    } else
    {
        mEncoder = std::make_shared<rev::CANEncoder>(*mMaster.get(), mConstants.kEncoderType, mConstants.kCountsPerRev);
    }
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetFeedbackDevice(*mEncoder.get()), ": Could not set Feedback Device : ");
    
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mEncoder->SetMeasurementPeriod(100), ": Could not set Measurement period : ");
    //set to ticks and ticks per 100 ms
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mEncoder->SetPositionConversionFactor(mConstants.kCountsPerRev), ": Could not set Position Conversion Factor : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mEncoder->SetVelocityConversionFactor(mConstants.kCountsPerRev/600.0), ": Could not set Velocity Conversion Factor : ");

    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->SetCANTimeout(100), ": Could not set CAN timeout : ");

    mForwardSoftLimit = (mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance;
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, mForwardSoftLimit), ": Could not set forward soft limit : ");

    mReverseSoftLimit = (mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance;
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, mReverseSoftLimit), ": Could not set reverse soft limit : ");

    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, mConstants.kEnableForwardSoftLimit), ": Could not enable forward soft limit : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, mConstants.kEnableReverseSoftLimit), ": Could not enable reverse soft limit : ");
    
    if (mConstants.kEnableVoltageCompensation)
    {
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->EnableVoltageCompensation(mConstants.kVoltageCompensation), ": Could not set voltage compensation : ");
    }

    //Number of PID slots - 1;
    for (int i =0; i < 3; i++)
    {
        std::string slot;
        if (i == 0)
        {
            slot = "MotionProfile";
        } else if (i == 1)
        {
            slot = "Position";
        } else if (i == 2)
        {
            slot = "Velocity";
        } else if (i == 3)
        {
            slot = "SmartMotion";
        } else 
        {
            std::cout<<"Spark Max set PIDF slot out of range" << mConstants.id << i << std::endl;
        }
        //Set all PIDF values for all slots.
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetP(mConstants.kP.at(i), i), ": Could not set kP " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetI(mConstants.kI.at(i), i), ": Could not set kI " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetD(mConstants.kD.at(i), i), ": Could not set kD " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetFF(mConstants.kF.at(i), i), ": Could not set kF " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetIZone(mConstants.kIZone.at(i), i), ": Could not set kIZone " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetIMaxAccum(mConstants.kMaxIAccum.at(i), i), ": Could not set kMaxIAccum " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetDFilter(mConstants.kDFilter.at(i), i), ": Could not set kDFilter " + slot + " : ");
    }

    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetSmartMotionMaxVelocity(mConstants.kMaxVelocity, Constants::kMotionMagicPIDSlot), ": Could not set SmartMotion Cruise Velocity : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetSmartMotionMaxAccel(mConstants.kMaxAcceleration, Constants::kMotionMagicPIDSlot), ": Could not set SmartMotion Max Acceleration : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mPIDController->SetSmartMotionAllowedClosedLoopError(mConstants.kAllowableClosedLoopError, Constants::kMotionMagicPIDSlot), ": Could not set SmartMotion Max error : ");

    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->SetClosedLoopRampRate(mConstants.kClosedLoopRampRate), ": Could not set Closed Loop Ramp Rate : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants.id, mMaster->SetOpenLoopRampRate(mConstants.kOpenLoopRampRate), ": Could not set Open Loop Ramp Rate : ");

    mMaster->SetInverted(mConstants.inverted);
    mMaster->SetIdleMode(mConstants.kIdleMode);

    for (int i = 0; i <mConstants.kSlaveIDs.size()-1; i++)
    {
        mSlaves.push_back(Drivers::SparkMaxFactory::createFollowerSparkMax(mConstants.kSlaveIDs.at(i).id, mMaster, mConstants.kMotorType));
        mSlaves.at(i)->SetInverted(mConstants.kSlaveIDs.at(i).invert_motor);
        mSlaves.at(i)->SetIdleMode(mConstants.kIdleMode);
    }

    stop();
}

void SparkMaxSubsystem::readPeriodicInputs()
{
    mPeriodicIO.timestamp = frc::Timer::GetFPGATimestamp();

    mPeriodicIO.reset_occurred = false;

    mMaster->ClearFaults();
    
    mPeriodicIO.error_ticks = 0.0;

    mPeriodicIO.position_ticks = (int)std::floor( mEncoder->GetPosition());
    mPeriodicIO.position_units = ticksToHomedUnits(mEncoder->GetPosition());

    mPeriodicIO.velocity_ticks_per_100ms = (int) std::floor( mEncoder->GetVelocity() );
    mPeriodicIO.velocity_units = ticksPer100msToUnitsPerSecond(mEncoder->GetVelocity());

    mPeriodicIO.output_percent = mMaster->GetAppliedOutput();
    mPeriodicIO.output_voltage = mMaster->GetAppliedOutput()*mMaster->GetBusVoltage();
    mPeriodicIO.master_current = mMaster->GetOutputCurrent();
    mPeriodicIO.error_ticks = 0.0;
    
    if (mControlState != ControlState::OPEN_LOOP)
    {
        mPeriodicIO.active_trajectory_position = mPeriodicIO.position_ticks;
        if (mPeriodicIO.active_trajectory_position < mReverseSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants.kName + ": Active Trajectory past reverse soft limit!");
        } else if (mPeriodicIO.active_trajectory_position > mForwardSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants.kName + ": Active Trajectory past forward soft limit!");
        } 

        int newVel = mPeriodicIO.velocity_ticks_per_100ms;
        if (util.epsilonEquals(newVel, mConstants.kMaxVelocity, std::max(1 , (int)std::floor(mConstants.kAllowableClosedLoopError))) || 
                util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, std::max(1, (int)std::floor(mConstants.kAllowableClosedLoopError))) )
        {
            mPeriodicIO.active_trajectory_acceleration = 0;
        } else 
        {
            mPeriodicIO.active_trajectory_acceleration = (newVel - mPeriodicIO.active_trajectory_velocity)* mConstants.kMaxAcceleration;//(mPeriodicIO.timestamp - prev_timestamp);
        }
        
        mPeriodicIO.active_trajectory_velocity = newVel;
            
    } else
    {
        mPeriodicIO.active_trajectory_acceleration = 0.0;
        mPeriodicIO.active_trajectory_velocity = 0.0;
        mPeriodicIO.active_trajectory_position = INFINITY;
    }

    // only absolute
    if (mConstants.kRecoverPositionOnReset)
    {
        mPeriodicIO.absolute_position = mPeriodicIO.position_ticks;
            mPeriodicIO.absolute_position_modded = mPeriodicIO.absolute_position % mConstants.kCountsPerRev;
            if (mPeriodicIO.absolute_position_modded < 0)
            {
                mPeriodicIO.absolute_position_modded += mConstants.kCountsPerRev;
            }

        
        if (prev_position_units == NAN)
        {
            prev_position_units = mPeriodicIO.position_units;
        } else 
        {
            mPeriodicIO.absolute_position = mPeriodicIO.position_ticks;
            mPeriodicIO.absolute_position_modded = mPeriodicIO.absolute_position % mConstants.kCountsPerRev;
            if (mPeriodicIO.absolute_position_modded < 0)
            {
                mPeriodicIO.absolute_position_modded += mConstants.kCountsPerRev;
            }

            //check this is actually accurate
            double est_disp = (mPeriodicIO.timestamp - prev_timestamp) * mPeriodicIO.velocity_ticks_per_100ms;
            mPeriodicIO.encoder_wraps += (mPeriodicIO.position_ticks - (prev_position_units + est_disp)) > 2000.0? -1 : (mPeriodicIO.position_ticks - (prev_position_units + est_disp)) < -2000.0 ? 1 : 0;
        }
    } else 
    {
        mPeriodicIO.encoder_wraps = 0;
        mPeriodicIO.absolute_position = 0;
        mPeriodicIO.absolute_position_modded = 0;
    }

    prev_position_units = mPeriodicIO.position_ticks;
    prev_timestamp = mPeriodicIO.timestamp;
}

void SparkMaxSubsystem::writePeriodicOutputs()
{
    if (mControlState == ControlState::MOTION_MAGIC)
    {
        mPIDController->SetReference(mPeriodicIO.demand, rev::ControlType::kSmartMotion, Constants::kMotionMagicPIDSlot, mPeriodicIO.feedforward);
    } else if (mControlState == ControlState::POSITION_PID)
    {
        mPIDController->SetReference(mPeriodicIO.demand, rev::ControlType::kPosition, Constants::kPositionPIDSlot, mPeriodicIO.feedforward);
    } else if (mControlState == ControlState::VELOCITY_PID)
    {
        mPIDController->SetReference(mPeriodicIO.demand, rev::ControlType::kVelocity, Constants::kVelocityPIDSlot, mPeriodicIO.feedforward);
    } else 
    {
        mPIDController->SetReference(mPeriodicIO.demand, rev::ControlType::kDutyCycle, 0, mPeriodicIO.feedforward);
    }
}

void SparkMaxSubsystem::zeroSensors()
{
    mPeriodicIO.absolute_offset = (int)std::floor(mEncoder->GetPosition());
    mEncoder->SetPosition(0.0);
    mHasBeenZeroed = true;
}

bool SparkMaxSubsystem::hasFinishedTrajectory()
{
    if (util.epsilonEquals(mPeriodicIO.active_trajectory_position, ticksToUnits(getSetpoint())), std::fmax(1.0, mConstants.kAllowableClosedLoopError))
    {
        return true;
    }
    return false;
}

double SparkMaxSubsystem::getActiveTrajectoryUnits()
{
    return ticksToHomedUnits(mPeriodicIO.active_trajectory_position);
}

double SparkMaxSubsystem::getActiveTrajectoryUnitsPerSecond()
{
    return ticksPer100msToUnitsPerSecond(mPeriodicIO.active_trajectory_velocity);
}

double SparkMaxSubsystem::getPredictedPositionUnits(double lookahead_secs)
{
    if (mControlState != ControlState::MOTION_MAGIC)
    {
        return getPosition();
    }

    double predicted_units = ticksToHomedUnits(mPeriodicIO.active_trajectory_position + 
        lookahead_secs * mPeriodicIO.active_trajectory_velocity + 
        .5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);
    if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position)
    {
        return std::fmin(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    } else
    {
        return std::fmax(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    }
    
}

bool SparkMaxSubsystem::atHomingLocation()
{
    return false;
}

void SparkMaxSubsystem::resetIfAtLimit()
{
    if (atHomingLocation())
    {
        zeroSensors();
    }
}

bool SparkMaxSubsystem::hasBeenZeroed()
{
    return mHasBeenZeroed;
}

void SparkMaxSubsystem::stop()
{
    setOpenLoop(0.0);
}

int SparkMaxSubsystem::estimateSensorPositionFromAbsolute()
{
    //figure out something better
    return getPositionTicks();
}

double SparkMaxSubsystem::getSetpoint()
{
    return (mControlState != ControlState::OPEN_LOOP)? ticksToUnits(mPeriodicIO.demand) : NAN;
}

void SparkMaxSubsystem::setSetpointMotionMagic(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::MOTION_MAGIC)
    {
        mControlState = ControlState::MOTION_MAGIC;
    }
}

void SparkMaxSubsystem::setSetpointMotionMagic(double units)
{
    setSetpointMotionMagic(units, 0.0);
}

void SparkMaxSubsystem::setSetpointPositionPID(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;
    if (mControlState != ControlState::POSITION_PID)
    {
        mControlState = ControlState::POSITION_PID;
    }
}

void SparkMaxSubsystem::setSetpointVelocityPID(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;
    if (mControlState != ControlState::VELOCITY_PID)
    {
        mControlState = ControlState::VELOCITY_PID;
    }
}

void SparkMaxSubsystem::setOpenLoop(double percentage)
{
    mPeriodicIO.demand = percentage;
    mPeriodicIO.feedforward = 0.0;
    if (mControlState != ControlState::OPEN_LOOP)
    {
        mControlState = ControlState::OPEN_LOOP;
    }
}

double SparkMaxSubsystem::ticksToUnits(double ticks)
{
    return ticks/mConstants.kTicksPerUnitDistance;
}

double SparkMaxSubsystem::ticksToHomedUnits(double ticks)
{
    double val = ticksToUnits(ticks);
    return val + mConstants.kHomePosition;
}

double SparkMaxSubsystem::unitsToTicks(double units)
{
    return units * mConstants.kTicksPerUnitDistance;
}

double SparkMaxSubsystem::homeAwareUnitsToTicks(double units)
{
    return unitsToTicks(units - mConstants.kHomePosition);
}

double SparkMaxSubsystem::constrainTicks(double ticks)
{
    return util.limit(ticks, mReverseSoftLimit, mForwardSoftLimit);
}

double SparkMaxSubsystem::ticksPer100msToUnitsPerSecond(double ticks_per_100ms)
{
    return ticksToUnits(ticks_per_100ms) * 10.0;
}

double SparkMaxSubsystem::unitsPerSecondToTicksPer100ms(double units_per_second)
{
    return unitsToTicks(units_per_second) /10.0;
}

TalonSRXSubsystem::TalonSRXSubsystem(TalonConstants constants)
{
    mConstants = constants;
    //if (mConstants.kIsTalonSRX)
    //{
        mMaster = Drivers::TalonFactory::createDefaultTalonSRX(mConstants.id);
    //} else 
    //{
        //mMaster = Drivers::TalonFactory::createDefaultTalonFX(mConstants.id);
    //}
    mForwardSoftLimit = ((mConstants.kMaxUnitsLimit - mConstants.kHomePosition)*mConstants.kTicksPerUnitDistance);    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigForwardSoftLimitThreshold((int) mForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure forward soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigForwardSoftLimitEnable(mConstants.kEnableForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable forward soft limit: ");

    mReverseSoftLimit = ((mConstants.kMinUnitsLimit - mConstants.kHomePosition)*mConstants.kTicksPerUnitDistance);
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigReverseSoftLimitThreshold((int) mReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure reverse soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigReverseSoftLimitEnable(mConstants.kEnableReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable reverse soft limit: ");

    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigVoltageCompSaturation(mConstants.kVoltageCompensation, Constants::kLongCANTimeoutMs), ": could not configure Voltage Compensation: ");

    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kP(Constants::kMotionProfileSlot, mConstants.kP.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kI(Constants::kMotionProfileSlot, mConstants.kI.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kD(Constants::kMotionProfileSlot, mConstants.kD.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kF(Constants::kMotionProfileSlot, mConstants.kF.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_IntegralZone(Constants::kMotionProfileSlot, mConstants.kIZone.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMaxIntegralAccumulator(Constants::kMotionProfileSlot, mConstants.kMaxIAccum.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigAllowableClosedloopError(Constants::kMotionProfileSlot, mConstants.kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kP(Constants::kPositionPIDSlot, mConstants.kP.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kI(Constants::kPositionPIDSlot, mConstants.kI.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kD(Constants::kPositionPIDSlot, mConstants.kD.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kF(Constants::kPositionPIDSlot, mConstants.kF.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_IntegralZone(Constants::kPositionPIDSlot, mConstants.kIZone.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMaxIntegralAccumulator(Constants::kPositionPIDSlot, mConstants.kMaxIAccum.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigAllowableClosedloopError(Constants::kPositionPIDSlot, mConstants.kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kP(Constants::kVelocityPIDSlot, mConstants.kP.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kI(Constants::kVelocityPIDSlot, mConstants.kI.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kD(Constants::kVelocityPIDSlot, mConstants.kD.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kF(Constants::kVelocityPIDSlot, mConstants.kF.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_IntegralZone(Constants::kVelocityPIDSlot, mConstants.kIZone.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMaxIntegralAccumulator(Constants::kVelocityPIDSlot, mConstants.kMaxIAccum.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigAllowableClosedloopError(Constants::kVelocityPIDSlot, mConstants.kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot Deadband: ");

    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMotionCruiseVelocity(mConstants.kMaxVelocity, Constants::kLongCANTimeoutMs), ": could not configure cruise velocity: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMotionAcceleration(mConstants.kMaxAcceleration, Constants::kLongCANTimeoutMs), ": could not configure acceleration: ");
    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigClosedloopRamp(mConstants.kClosedLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure closed loop ramp rate: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigOpenloopRamp(mConstants.kOpenLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure open loop ramp rate: ");

    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigContinuousCurrentLimit(mConstants.kContinuousCurrentLimit, Constants::kLongCANTimeoutMs), ": could not configure continuous current limit: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigPeakCurrentLimit(mConstants.kPeakCurrentLimit, Constants::kLongCANTimeoutMs), ": could not configure peak current limit: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigPeakCurrentDuration(mConstants.kPeakCurrentDuration, Constants::kLongCANTimeoutMs), ": could not configure peak current duration: ");

    
    mMaster->EnableVoltageCompensation(mConstants.kEnableVoltageCompensation);
    mMaster->SetInverted(mConstants.inverted);
    mMaster->SetSensorPhase(mConstants.kInvertSensorPhase);
    mMaster->SetNeutralMode(mConstants.kNeutralMode);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);

    for (int i = 0; i < mConstants.kSlaveIDs.size()-1; i++)
    {
        //if (mConstants.kSlaveIDs.at(i).isTalonSRX)
        //{
            mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonSRX(mConstants.kSlaveIDs.at(i).id, mMaster));
        //} //else 
        //{
        //    mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonFX(mConstants.kSlaveIDs.at(i).id, mMaster));
        //}
        mSlaves.at(i)->SetInverted(mConstants.kSlaveIDs.at(i).invert_motor);
        mSlaves.at(i)->SetNeutralMode(mConstants.kNeutralMode);
        
        
    }
}
 
void TalonSRXSubsystem::readPeriodicInputs()
{
    mPeriodicIO.timestamp = frc::Timer::GetFPGATimestamp();

    if ( mMaster->HasResetOccurred() ) 
    {
        frc::DriverStation::ReportError(mConstants.kName + ": Talon Reset! ");
        mPeriodicIO.reset_occurred = true;
        return;
    } else
    {
        mPeriodicIO.reset_occurred = false;
    }

    mMaster->GetStickyFaults(mFaults);
    if (mFaults.HasAnyFault())
    {
        frc::DriverStation::ReportError(mConstants.kName + ": Talon Fault! " + mFaults.ToString());
        mMaster->ClearStickyFaults();
    }

    if (mMaster->GetControlMode() == ControlMode::MotionMagic)
    {
        mPeriodicIO.active_trajectory_position = mMaster->GetActiveTrajectoryPosition();

        if (mPeriodicIO.active_trajectory_position < mReverseSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants.kName + ": Active Trajectory Past Reverse soft limit!");
        } else 
        {
            frc::DriverStation::ReportError(mConstants.kName + ": Active Trajectory Past Forward soft limit!");
        }
        int newVel = mMaster->GetActiveTrajectoryVelocity();
        if (util.epsilonEquals(newVel, mConstants.kMaxVelocity, std::fmax(1.0, mConstants.kAllowableClosedLoopError)) || 
            util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, std::fmax(1.0, mConstants.kAllowableClosedLoopError)))
        {
            //Mechanism is at a constant velocity (velocity is at max or didn't change)
            mPeriodicIO.active_trajectory_acceleration = 0.0;
        } else
        {
            mPeriodicIO.active_trajectory_acceleration = (newVel - mPeriodicIO.active_trajectory_velocity) * mConstants.kMaxAcceleration;
        }
        mPeriodicIO.active_trajectory_velocity = newVel;
    } else
    {
        mPeriodicIO.active_trajectory_position = 0;
        mPeriodicIO.active_trajectory_velocity = 0;
        mPeriodicIO.active_trajectory_acceleration = 0;
    }
    if (mMaster->GetControlMode() == ControlMode::Position)
    {
        mPeriodicIO.error_ticks = mMaster->GetClosedLoopError();
    } else
    {
        mPeriodicIO.error_ticks = 0;
    }
    mPeriodicIO.master_current = mMaster->GetOutputCurrent();
    mPeriodicIO.output_voltage = mMaster->GetMotorOutputVoltage();
    mPeriodicIO.output_percent = mMaster->GetMotorOutputPercent();
    mPeriodicIO.position_ticks = mMaster->GetSelectedSensorPosition();
    mPeriodicIO.position_units = ticksToHomedUnits(mPeriodicIO.position_ticks);
    mPeriodicIO.velocity_ticks_per_100ms = mMaster->GetSelectedSensorVelocity();
    mPeriodicIO.velocity_units = ticksPer100msToUnitsPerSecond(mPeriodicIO.velocity_ticks_per_100ms);
    
    if (mConstants.kRecoverPositionOnReset)
    {
        mPeriodicIO.absolute_position = mMaster->GetSensorCollection().GetPulseWidthPosition();
        mPeriodicIO.absolute_position_modded = mPeriodicIO.absolute_position % 4096;
        if (mPeriodicIO.absolute_position_modded < 0)
        {
            mPeriodicIO.absolute_position_modded += 4096;
        }

        int estimated_pulsed_pos = ((mConstants.invert_sensor_phase ? -1 : 1)*mPeriodicIO.position_ticks) + mPeriodicIO.absolute_offset;
        int new_wraps = (int) std::floor(estimated_pulsed_pos/4096.0);

        if (std::abs(mPeriodicIO.encoder_wraps - new_wraps) <= 1 )
        {
            mPeriodicIO.encoder_wraps = new_wraps;
        }
    } else 
    {
        mPeriodicIO.absolute_position = 0.0;
        mPeriodicIO.absolute_position_modded = 0.0;
    }
}

 
void TalonSRXSubsystem::writePeriodicOutputs()
{
    if (mControlState == ControlState::MOTION_MAGIC)
    {
        mMaster->Set(ControlMode::MotionMagic, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    } else if (mControlState == ControlState::POSITION_PID || mControlState == ControlState::MOTION_PROFILING)
    {
        mMaster->Set(ControlMode::Position, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    } else if (mControlState == ControlState::VELOCITY_PID)
    {
        mMaster->Set(ControlMode::Velocity, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    } else
    {
        mMaster->Set(ControlMode::PercentOutput, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    }
}

 
void TalonSRXSubsystem::zeroSensors()
{
    mMaster->SetSelectedSensorPosition(0.0, Constants::kCANTimeoutMs);
    mPeriodicIO.absolute_offset = getAbsoluteEncoderRawPosition(mMaster->GetSensorCollection().GetPulseWidthPosition());
    mHasBeenZeroed = true;
}

 
void TalonSRXSubsystem::OnStart(double timestamp){}

 
void TalonSRXSubsystem::OnLoop(double timestamp)
{
    if (mPeriodicIO.reset_occurred)
    {
        std::cout << mConstants.kName << " : Master Talon reset occurred; resetting frame rates."<<std::endl;
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);
    
        if (mConstants.kRecoverPositionOnReset)
        {
            mMaster->SetSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants::kCANTimeoutMs);
        }
    }
    handleMasterReset(mPeriodicIO.reset_occurred);
    for (auto slave : mSlaves)
    {
        if (slave->HasResetOccurred())
        {
            std::cout << mConstants.kName << " : Slave Talon reset occurred."<<std::endl;
        }
    }
}

 
void TalonSRXSubsystem::OnStop(double timestamp)
{
    stop();
}

 
bool TalonSRXSubsystem::hasFinishedTrajectory()
{
    if (util.epsilonEquals((double) mPeriodicIO.active_trajectory_position, ticksToUnits(getSetpoint()), std::fmax(1.0, mConstants.kAllowableClosedLoopError)))
    {
        return true;
    }
    return false;
}

 
double TalonSRXSubsystem::getActiveTrajectoryUnits()
{
    return ticksToHomedUnits(mPeriodicIO.active_trajectory_position);
}

 
double TalonSRXSubsystem::getActiveTrajectoryUnitsPerSecond()
{
    return ticksPer100msToUnitsPerSecond(mPeriodicIO.active_trajectory_velocity);
}

 
double TalonSRXSubsystem::getPredictedPositionUnits(double lookahead_secs)
{
    if (mMaster->GetControlMode() != ControlMode::MotionMagic)
    {
        return getPosition();
    }

    double predicted_units = ticksToHomedUnits(mPeriodicIO.active_trajectory_position +
            lookahead_secs * mPeriodicIO.active_trajectory_velocity +
            mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);
    if(mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position)
    {
        return std::fmin(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    } else
    {
        return std::fmax(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    }
}

 
bool TalonSRXSubsystem::atHomingLocation()
{
    return false;
}

 
void TalonSRXSubsystem::resetIfAtLimit()
{
    if (atHomingLocation())
    {
        zeroSensors();
    }
}

 
int TalonSRXSubsystem::getAbsoluteEncoderRawPosition(int pulse_width_position)
{
    int abs_raw_with_rollover = pulse_width_position % 4096;
    return abs_raw_with_rollover + (abs_raw_with_rollover < 0 ? abs_raw_with_rollover + 4096 : 0);
}

 
bool TalonSRXSubsystem::hasBeenZeroed()
{
    return mHasBeenZeroed;
}

 
void TalonSRXSubsystem::stop()
{
    setOpenLoop(0.0);
}

 
int TalonSRXSubsystem::estimateSensorPositionFromAbsolute()
{
    int estimated_pulse_pos = (mPeriodicIO.encoder_wraps * 4096) + mPeriodicIO.absolute_position_modded;
    int estimate_position_ticks = (mConstants.invert_sensor_phase ? -1 : 1) * (estimated_pulse_pos -mPeriodicIO.absolute_offset);
    return estimate_position_ticks;
}

 
double TalonSRXSubsystem::getSetpoint()
{
    return (mControlState != ControlState::OPEN_LOOP) ? ticksToUnits(mPeriodicIO.demand) : NAN;
}

 
void TalonSRXSubsystem::setSetpointMotionMagic(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::MOTION_MAGIC)
    {
        mControlState = ControlState::MOTION_MAGIC;
    }
}

 
void TalonSRXSubsystem::setSetpointMotionMagic(double units)
{
    setSetpointMotionMagic(units, 0.0);
}

 
void TalonSRXSubsystem::setSetpointPositionPID(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::POSITION_PID)
    {
        mControlState = ControlState::POSITION_PID;
    }
}

 
void TalonSRXSubsystem::setSetpointVelocityPID(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::VELOCITY_PID)
    {
        mControlState = ControlState::VELOCITY_PID;
    }
}

 
void TalonSRXSubsystem::setOpenLoop(double percentage)
{
    mPeriodicIO.demand = percentage;
    if (mControlState != ControlState::OPEN_LOOP)
    {
        mControlState = ControlState::OPEN_LOOP;
    }
}

 
double TalonSRXSubsystem::ticksToUnits(double ticks)
{
    return ticks/mConstants.kTicksPerUnitDistance;
}

 
double TalonSRXSubsystem::ticksToHomedUnits(double ticks)
{
    double val = ticksToUnits(ticks);
    return val + mConstants.kHomePosition;
}

 
double TalonSRXSubsystem::unitsToTicks(double units)
{
    return units * mConstants.kTicksPerUnitDistance;
}

 
double TalonSRXSubsystem::homeAwareUnitsToTicks(double units)
{
    return unitsToTicks(units-mConstants.kHomePosition);
}

 
double TalonSRXSubsystem::constrainTicks(double ticks)
{
    return util.limit(ticks, mReverseSoftLimit, mForwardSoftLimit);
}

 
double TalonSRXSubsystem::ticksPer100msToUnitsPerSecond(double ticks_per_100ms)
{
    return ticksToUnits(ticks_per_100ms) * 10.0;
}

 
double TalonSRXSubsystem::unitsPerSecondToTicksPer100ms(double units_per_second)
{
    return unitsToTicks(units_per_second) / 10.0;
}

TalonFXSubsystem::TalonFXSubsystem(TalonConstants constants)
{
    mConstants = constants;
    //if (mConstants.kIsTalonSRX)
    //{
        mMaster = Drivers::TalonFactory::createDefaultTalonFX(mConstants.id);
    //} else 
    //{
        //mMaster = Drivers::TalonFactory::createDefaultTalonFX(mConstants.id);
    //}
    mForwardSoftLimit = ((mConstants.kMaxUnitsLimit - mConstants.kHomePosition)*mConstants.kTicksPerUnitDistance);    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigForwardSoftLimitThreshold((int) mForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure forward soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigForwardSoftLimitEnable(mConstants.kEnableForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable forward soft limit: ");

    mReverseSoftLimit = ((mConstants.kMinUnitsLimit - mConstants.kHomePosition)*mConstants.kTicksPerUnitDistance);
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigReverseSoftLimitThreshold((int) mReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure reverse soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigReverseSoftLimitEnable(mConstants.kEnableReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable reverse soft limit: ");

    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigVoltageCompSaturation(mConstants.kVoltageCompensation, Constants::kLongCANTimeoutMs), ": could not configure Voltage Compensation: ");

    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kP(Constants::kMotionProfileSlot, mConstants.kP.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kI(Constants::kMotionProfileSlot, mConstants.kI.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kD(Constants::kMotionProfileSlot, mConstants.kD.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kF(Constants::kMotionProfileSlot, mConstants.kF.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_IntegralZone(Constants::kMotionProfileSlot, mConstants.kIZone.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMaxIntegralAccumulator(Constants::kMotionProfileSlot, mConstants.kMaxIAccum.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigAllowableClosedloopError(Constants::kMotionProfileSlot, mConstants.kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kP(Constants::kPositionPIDSlot, mConstants.kP.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kI(Constants::kPositionPIDSlot, mConstants.kI.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kD(Constants::kPositionPIDSlot, mConstants.kD.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kF(Constants::kPositionPIDSlot, mConstants.kF.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_IntegralZone(Constants::kPositionPIDSlot, mConstants.kIZone.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMaxIntegralAccumulator(Constants::kPositionPIDSlot, mConstants.kMaxIAccum.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigAllowableClosedloopError(Constants::kPositionPIDSlot, mConstants.kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kP(Constants::kVelocityPIDSlot, mConstants.kP.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kI(Constants::kVelocityPIDSlot, mConstants.kI.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kD(Constants::kVelocityPIDSlot, mConstants.kD.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_kF(Constants::kVelocityPIDSlot, mConstants.kF.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->Config_IntegralZone(Constants::kVelocityPIDSlot, mConstants.kIZone.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMaxIntegralAccumulator(Constants::kVelocityPIDSlot, mConstants.kMaxIAccum.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigAllowableClosedloopError(Constants::kVelocityPIDSlot, mConstants.kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot Deadband: ");

    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMotionCruiseVelocity(mConstants.kMaxVelocity, Constants::kLongCANTimeoutMs), ": could not configure cruise velocity: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigMotionAcceleration(mConstants.kMaxAcceleration, Constants::kLongCANTimeoutMs), ": could not configure acceleration: ");
    
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigClosedloopRamp(mConstants.kClosedLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure closed loop ramp rate: ");
    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigOpenloopRamp(mConstants.kOpenLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure open loop ramp rate: ");

//    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigContinuousCurrentLimit(mConstants.kContinuousCurrentLimit, Constants::kLongCANTimeoutMs), ": could not configure continuous current limit: ");
//    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigPeakCurrentLimit(mConstants.kPeakCurrentLimit, Constants::kLongCANTimeoutMs), ": could not configure peak current limit: ");
//    Drivers::TalonFactory::handleCANError(mConstants.id, mMaster->ConfigPeakCurrentDuration(mConstants.kPeakCurrentDuration, Constants::kLongCANTimeoutMs), ": could not configure peak current duration: ");

    
    mMaster->EnableVoltageCompensation(mConstants.kEnableVoltageCompensation);
    mMaster->SetInverted(mConstants.inverted);
    mMaster->SetSensorPhase(mConstants.kInvertSensorPhase);
    mMaster->SetNeutralMode(mConstants.kNeutralMode);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);

    for (int i = 0; i < mConstants.kSlaveIDs.size()-1; i++)
    {
        //if (mConstants.kSlaveIDs.at(i).isTalonSRX)
        //{
            mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonFX(mConstants.kSlaveIDs.at(i).id, mMaster));
        //} //else 
        //{
        //    mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonFX(mConstants.kSlaveIDs.at(i).id, mMaster));
        //}
        mSlaves.at(i)->SetInverted(mConstants.kSlaveIDs.at(i).invert_motor);
        mSlaves.at(i)->SetNeutralMode(mConstants.kNeutralMode);
        
        
    }
}
 
void TalonFXSubsystem::readPeriodicInputs()
{
    mPeriodicIO.timestamp = frc::Timer::GetFPGATimestamp();

    if ( mMaster->HasResetOccurred() ) 
    {
        frc::DriverStation::ReportError(mConstants.kName + ": Talon Reset! ");
        mPeriodicIO.reset_occurred = true;
        return;
    } else
    {
        mPeriodicIO.reset_occurred = false;
    }

    mMaster->GetStickyFaults(mFaults);
    if (mFaults.HasAnyFault())
    {
        frc::DriverStation::ReportError(mConstants.kName + ": Talon Fault! " + mFaults.ToString());
        mMaster->ClearStickyFaults();
    }

    if (mMaster->GetControlMode() == ControlMode::MotionMagic)
    {
        mPeriodicIO.active_trajectory_position = mMaster->GetActiveTrajectoryPosition();

        if (mPeriodicIO.active_trajectory_position < mReverseSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants.kName + ": Active Trajectory Past Reverse soft limit!");
        } else 
        {
            frc::DriverStation::ReportError(mConstants.kName + ": Active Trajectory Past Forward soft limit!");
        }
        int newVel = mMaster->GetActiveTrajectoryVelocity();
        if (util.epsilonEquals(newVel, mConstants.kMaxVelocity, std::fmax(1.0, mConstants.kAllowableClosedLoopError)) || 
            util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, std::fmax(1.0, mConstants.kAllowableClosedLoopError)))
        {
            //Mechanism is at a constant velocity (velocity is at max or didn't change)
            mPeriodicIO.active_trajectory_acceleration = 0.0;
        } else
        {
            mPeriodicIO.active_trajectory_acceleration = (newVel - mPeriodicIO.active_trajectory_velocity) * mConstants.kMaxAcceleration;
        }
        mPeriodicIO.active_trajectory_velocity = newVel;
    } else
    {
        mPeriodicIO.active_trajectory_position = 0;
        mPeriodicIO.active_trajectory_velocity = 0;
        mPeriodicIO.active_trajectory_acceleration = 0;
    }
    if (mMaster->GetControlMode() == ControlMode::Position)
    {
        mPeriodicIO.error_ticks = mMaster->GetClosedLoopError();
    } else
    {
        mPeriodicIO.error_ticks = 0;
    }
    mPeriodicIO.master_current = mMaster->GetOutputCurrent();
    mPeriodicIO.output_voltage = mMaster->GetMotorOutputVoltage();
    mPeriodicIO.output_percent = mMaster->GetMotorOutputPercent();
    mPeriodicIO.position_ticks = mMaster->GetSelectedSensorPosition();
    mPeriodicIO.position_units = ticksToHomedUnits(mPeriodicIO.position_ticks);
    mPeriodicIO.velocity_ticks_per_100ms = mMaster->GetSelectedSensorVelocity();
    mPeriodicIO.velocity_units = ticksPer100msToUnitsPerSecond(mPeriodicIO.velocity_ticks_per_100ms);
    /*
    if (mConstants.kRecoverPositionOnReset)
    {
        mPeriodicIO.absolute_position = mMaster->GetSensorCollection().GetPulseWidthPosition();
        mPeriodicIO.absolute_position_modded = mPeriodicIO.absolute_position % 4096;
        if (mPeriodicIO.absolute_position_modded < 0)
        {
            mPeriodicIO.absolute_position_modded += 4096;
        }

        int estimated_pulsed_pos = ((mConstants.invert_sensor_phase ? -1 : 1)*mPeriodicIO.position_ticks) + mPeriodicIO.absolute_offset;
        int new_wraps = (int) std::floor(estimated_pulsed_pos/4096.0);

        if (std::abs(mPeriodicIO.encoder_wraps - new_wraps) <= 1 )
        {
            mPeriodicIO.encoder_wraps = new_wraps;
        }
    } else 
    {
        */
        mPeriodicIO.absolute_position = 0.0;
        mPeriodicIO.absolute_position_modded = 0.0;
    //}
}

 
void TalonFXSubsystem::writePeriodicOutputs()
{
    if (mControlState == ControlState::MOTION_MAGIC)
    {
        mMaster->Set(ControlMode::MotionMagic, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    } else if (mControlState == ControlState::POSITION_PID || mControlState == ControlState::MOTION_PROFILING)
    {
        mMaster->Set(ControlMode::Position, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    } else if (mControlState == ControlState::VELOCITY_PID)
    {
        mMaster->Set(ControlMode::Velocity, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    } else
    {
        mMaster->Set(ControlMode::PercentOutput, mPeriodicIO.demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO.feedforward);
    }
}

 
void TalonFXSubsystem::zeroSensors()
{
    mMaster->SetSelectedSensorPosition(0.0, Constants::kCANTimeoutMs);
    //mPeriodicIO.absolute_offset = getAbsoluteEncoderRawPosition(mMaster->GetSensorCollection().GetPulseWidthPosition());
    mHasBeenZeroed = true;
}

 
void TalonFXSubsystem::OnStart(double timestamp){}

 
void TalonFXSubsystem::OnLoop(double timestamp)
{
    if (mPeriodicIO.reset_occurred)
    {
        std::cout << mConstants.kName << " : Master Talon reset occurred; resetting frame rates."<<std::endl;
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants.kStatusFrame8UpdateRate, 20);
    
        if (mConstants.kRecoverPositionOnReset)
        {
            mMaster->SetSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants::kCANTimeoutMs);
        }
    }
    handleMasterReset(mPeriodicIO.reset_occurred);
    for (auto slave : mSlaves)
    {
        if (slave->HasResetOccurred())
        {
            std::cout << mConstants.kName << " : Slave Talon reset occurred."<<std::endl;
        }
    }
}

 
void TalonFXSubsystem::OnStop(double timestamp)
{
    stop();
}

 
bool TalonFXSubsystem::hasFinishedTrajectory()
{
    if (util.epsilonEquals((double) mPeriodicIO.active_trajectory_position, ticksToUnits(getSetpoint()), std::fmax(1.0, mConstants.kAllowableClosedLoopError)))
    {
        return true;
    }
    return false;
}

 
double TalonFXSubsystem::getActiveTrajectoryUnits()
{
    return ticksToHomedUnits(mPeriodicIO.active_trajectory_position);
}

 
double TalonFXSubsystem::getActiveTrajectoryUnitsPerSecond()
{
    return ticksPer100msToUnitsPerSecond(mPeriodicIO.active_trajectory_velocity);
}

 
double TalonFXSubsystem::getPredictedPositionUnits(double lookahead_secs)
{
    if (mMaster->GetControlMode() != ControlMode::MotionMagic)
    {
        return getPosition();
    }

    double predicted_units = ticksToHomedUnits(mPeriodicIO.active_trajectory_position +
            lookahead_secs * mPeriodicIO.active_trajectory_velocity +
            mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);
    if(mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position)
    {
        return std::fmin(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    } else
    {
        return std::fmax(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
    }
}

 
bool TalonFXSubsystem::atHomingLocation()
{
    return false;
}

 
void TalonFXSubsystem::resetIfAtLimit()
{
    if (atHomingLocation())
    {
        zeroSensors();
    }
}

 
int TalonFXSubsystem::getAbsoluteEncoderRawPosition(int pulse_width_position)
{
    int abs_raw_with_rollover = pulse_width_position % 4096;
    return abs_raw_with_rollover + (abs_raw_with_rollover < 0 ? abs_raw_with_rollover + 4096 : 0);
}

 
bool TalonFXSubsystem::hasBeenZeroed()
{
    return mHasBeenZeroed;
}

 
void TalonFXSubsystem::stop()
{
    setOpenLoop(0.0);
}

 
int TalonFXSubsystem::estimateSensorPositionFromAbsolute()
{
    int estimated_pulse_pos = (mPeriodicIO.encoder_wraps * 4096) + mPeriodicIO.absolute_position_modded;
    int estimate_position_ticks = (mConstants.invert_sensor_phase ? -1 : 1) * (estimated_pulse_pos -mPeriodicIO.absolute_offset);
    return estimate_position_ticks;
}

 
double TalonFXSubsystem::getSetpoint()
{
    return (mControlState != ControlState::OPEN_LOOP) ? ticksToUnits(mPeriodicIO.demand) : NAN;
}

 
void TalonFXSubsystem::setSetpointMotionMagic(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::MOTION_MAGIC)
    {
        mControlState = ControlState::MOTION_MAGIC;
    }
}

 
void TalonFXSubsystem::setSetpointMotionMagic(double units)
{
    setSetpointMotionMagic(units, 0.0);
}

 
void TalonFXSubsystem::setSetpointPositionPID(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::POSITION_PID)
    {
        mControlState = ControlState::POSITION_PID;
    }
}

 
void TalonFXSubsystem::setSetpointVelocityPID(double units, double feedforward_v)
{
    mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants.kF.at(Constants::kMotionMagicPIDSlot) + mConstants.kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::VELOCITY_PID)
    {
        mControlState = ControlState::VELOCITY_PID;
    }
}

 
void TalonFXSubsystem::setOpenLoop(double percentage)
{
    mPeriodicIO.demand = percentage;
    if (mControlState != ControlState::OPEN_LOOP)
    {
        mControlState = ControlState::OPEN_LOOP;
    }
}

 
double TalonFXSubsystem::ticksToUnits(double ticks)
{
    return ticks/mConstants.kTicksPerUnitDistance;
}

 
double TalonFXSubsystem::ticksToHomedUnits(double ticks)
{
    double val = ticksToUnits(ticks);
    return val + mConstants.kHomePosition;
}

 
double TalonFXSubsystem::unitsToTicks(double units)
{
    return units * mConstants.kTicksPerUnitDistance;
}

 
double TalonFXSubsystem::homeAwareUnitsToTicks(double units)
{
    return unitsToTicks(units-mConstants.kHomePosition);
}

 
double TalonFXSubsystem::constrainTicks(double ticks)
{
    return util.limit(ticks, mReverseSoftLimit, mForwardSoftLimit);
}

 
double TalonFXSubsystem::ticksPer100msToUnitsPerSecond(double ticks_per_100ms)
{
    return ticksToUnits(ticks_per_100ms) * 10.0;
}

 
double TalonFXSubsystem::unitsPerSecondToTicksPer100ms(double units_per_second)
{
    return unitsToTicks(units_per_second) / 10.0;
}