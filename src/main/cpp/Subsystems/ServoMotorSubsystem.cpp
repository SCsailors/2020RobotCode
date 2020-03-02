/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/ServoMotorSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
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
    return (double) mPeriodicIO->position_ticks;
}

//units
double ServoMotorSubsystem::getPosition()
{
    return mPeriodicIO->position_units;
}

//units/sec
double ServoMotorSubsystem::getVelocity()
{
    return mPeriodicIO->velocity_units;
}

SparkMaxSubsystem::SparkMaxSubsystem(std::shared_ptr<SparkMaxConstants> constants) 
{
    #ifdef CompetitionBot
    mConstants = constants;
    
    mModeChooser.SetDefaultOption("OPEN_LOOP", ControlState::OPEN_LOOP);
    mModeChooser.AddOption("MAGIC_MOTION", ControlState::MOTION_MAGIC);
    mModeChooser.AddOption("POSITION_PID", ControlState::POSITION_PID);
    mModeChooser.AddOption("VELOCITY_PID", ControlState::VELOCITY_PID);
    //mModeChooser.AddOption("MOTION_PROFILE", Modes::MOTIONPROFILE);
    frc::SmartDashboard::PutData("Subsystems/" + mConstants->kName + "/PIDTuning/ Mode", &mModeChooser);
    
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Demand: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Feedforward: ", 0.0);
    frc::SmartDashboard::PutBoolean("Subsystems/" + mConstants->kName + "/PIDTuning/ Enabled: ", false);

    kp = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", 0.0);
    ki = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", 0.0);
    kd = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", 0.0);
    kf = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", 0.0);
    
    std::cout << "Starting Spark Max Subsystem: " << mConstants->id << std::endl;
    mMaster = Drivers::SparkMaxFactory::createDefaultSparkMax(mConstants->id, mConstants->kMotorType);

    mPIDController = std::make_shared<rev::CANPIDController>(mMaster->GetPIDController());

    if (mConstants->kEnableForwardLimitSwitch)
    {
        mForwardLimitSwitch = std::make_shared<rev::CANDigitalInput>(*mMaster.get(), rev::CANDigitalInput::LimitSwitch::kForward, mConstants->kForwardLimitSwitchPolarity);
    }
    
    if (mConstants->kEnableReverseLimitSwitch)
    {
        mReverseLimitSwitch = std::make_shared<rev::CANDigitalInput>(*mMaster.get(), rev::CANDigitalInput::LimitSwitch::kReverse, mConstants->kReverseLimitSwitchPolarity);
    }

    if (mConstants->kIsAltEncoder)
    {
        mEncoder = std::make_shared<rev::CANEncoder>(*mMaster.get(), mConstants->kAltEncoderType, mConstants->kCountsPerRev);
    } else
    {
        mEncoder = std::make_shared<rev::CANEncoder>(*mMaster.get(), mConstants->kEncoderType, mConstants->kCountsPerRev);
    }
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetFeedbackDevice(*mEncoder.get()), ": Could not set Feedback Device : ");
    
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mEncoder->SetMeasurementPeriod(100), ": Could not set Measurement period : ");
    //set to ticks and ticks per 100 ms
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mEncoder->SetPositionConversionFactor(1.0), ": Could not set Position Conversion Factor : "); //none
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mEncoder->SetVelocityConversionFactor(1.0/60.0), ": Could not set Velocity Conversion Factor : "); //rpm -> rotations/second

    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->SetCANTimeout(100), ": Could not set CAN timeout : ");

    mForwardSoftLimit = (mConstants->kMaxUnitsLimit - mConstants->kHomePosition) * mConstants->kTicksPerUnitDistance;
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Forward Soft Limit Position raw: ", mForwardSoftLimit);
    if (mConstants->kEnableForwardSoftLimit)
    {
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, mForwardSoftLimit), ": Could not set forward soft limit : ");
    }
    
    mReverseSoftLimit = (mConstants->kMinUnitsLimit - mConstants->kHomePosition) * mConstants->kTicksPerUnitDistance;
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Reverse Soft Limit Position raw: ", mReverseSoftLimit);
    if (mConstants->kEnableReverseSoftLimit)
    {
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, mReverseSoftLimit), ": Could not set reverse soft limit : ");
    }
    
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, mConstants->kEnableForwardSoftLimit), ": Could not enable forward soft limit : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, mConstants->kEnableReverseSoftLimit), ": Could not enable reverse soft limit : ");
    
    if (mConstants->kEnableVoltageCompensation)
    {
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->EnableVoltageCompensation(mConstants->kVoltageCompensation), ": Could not set voltage compensation : ");
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
            std::cout<<"Spark Max set PIDF slot out of range" << mConstants->id << i << std::endl;
        }
        //Set all PIDF values for all slots.
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetP(mConstants->kP.at(i), i), ": Could not set kP " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetI(mConstants->kI.at(i), i), ": Could not set kI " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetD(mConstants->kD.at(i), i), ": Could not set kD " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetFF(mConstants->kF.at(i), i), ": Could not set kF " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetIZone(mConstants->kIZone.at(i), i), ": Could not set kIZone " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetIMaxAccum(mConstants->kMaxIAccum.at(i), i), ": Could not set kMaxIAccum " + slot + " : ");
        Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetDFilter(mConstants->kDFilter.at(i), i), ": Could not set kDFilter " + slot + " : ");
    }

    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetSmartMotionMaxVelocity(mConstants->kMaxVelocity, Constants::kMotionMagicPIDSlot), ": Could not set SmartMotion Cruise Velocity : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetSmartMotionMaxAccel(mConstants->kMaxAcceleration, Constants::kMotionMagicPIDSlot), ": Could not set SmartMotion Max Acceleration : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mPIDController->SetSmartMotionAllowedClosedLoopError(mConstants->kAllowableClosedLoopError, Constants::kMotionMagicPIDSlot), ": Could not set SmartMotion Max error : ");

    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->SetClosedLoopRampRate(mConstants->kClosedLoopRampRate), ": Could not set Closed Loop Ramp Rate : ");
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->SetOpenLoopRampRate(mConstants->kOpenLoopRampRate), ": Could not set Open Loop Ramp Rate : ");

    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->SetSecondaryCurrentLimit(mConstants->kSecondaryCurrentLimit), ": Could not set Secondary Current Limit: ");
    Drivers::SparkMaxFactory::handleCANError(mConstants->id, mMaster->SetSmartCurrentLimit(mConstants->kCurrentStallLimit, mConstants->kCurrentStallLimit), ": Could not set Smart Current Limit: ");

    mMaster->SetInverted(mConstants->inverted);
    mMaster->SetIdleMode(mConstants->kIdleMode);
    if (mConstants->kSlaveIDs.size() == 0)
    {
        std::cout<<"SparkMaxSubsystem: skipping slave"<<std::endl;
    } else
    {
        
    for (auto slave: mConstants->kSlaveIDs)
    {
        if ((slave->id) == -1)
        {
            continue;
        }
        mSlaves.push_back(Drivers::SparkMaxFactory::createFollowerSparkMax(slave->id, mMaster, mConstants->kMotorType));
        mSlaves.at(i)->SetInverted(mConstants->kSlaveIDs.at(i)->invert_motor);
        mSlaves.at(i)->SetIdleMode(mConstants->kIdleMode);
        i++;
    }        
    }
    


    stop();
    #endif
}

void SparkMaxSubsystem::readPeriodicInputs()
{
    
    //std::cout << "SparkMaxSubsystem: " << mConstants->kName << ": read periodic inputs" << std::endl;
    mPeriodicIO->timestamp = frc::Timer::GetFPGATimestamp();

    mPeriodicIO->reset_occurred = false;

    //mMaster->ClearFaults();
    #ifdef CompetitionBot
        mPeriodicIO->error_ticks = 0.0;
        if (mConstants->readInputs)
        {
            mPeriodicIO->position_ticks = (int) std::round( mEncoder->GetPosition());
        mPeriodicIO->position_units = ticksToHomedUnits(mEncoder->GetPosition());

        mPeriodicIO->velocity_ticks_per_100ms = (int) std::round( mEncoder->GetVelocity() );
        mPeriodicIO->velocity_units = ticksPer100msToUnitsPerSecond(mEncoder->GetVelocity());

        mPeriodicIO->output_percent = mMaster->GetAppliedOutput();
        mPeriodicIO->output_voltage = mMaster->GetAppliedOutput()*mMaster->GetBusVoltage();
        mPeriodicIO->master_current = mMaster->GetOutputCurrent();
        mPeriodicIO->error_ticks = 0.0;

        }

        
    mPIDMode = mModeChooser.GetSelected();
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual mode: ", mControlState);
    k++;

    int i = mControlState;
    if (i < 4 ){
    
        p = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", mConstants->kP.at(i));
        i = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", mConstants->kI.at(i));
        d = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", mConstants->kD.at(i));
        f = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", mConstants->kF.at(i));
    
    
        
    

     
        if (k % 10 == 0)
        {
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kP: ", mPIDController->GetP(i));
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kI: ", mPIDController->GetI(i));
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kD: ", mPIDController->GetD(i));
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kF: ", mPIDController->GetFF(i));
            

        }
    }
       #endif
    #ifndef CompetitionBot
        mPeriodicIO->error_ticks = 0.0;

        mPeriodicIO->position_ticks = 0;
        mPeriodicIO->position_units = 0.0;

        mPeriodicIO->velocity_ticks_per_100ms = 0;
        mPeriodicIO->velocity_units = 0.0;

        mPeriodicIO->output_percent = 0.0;
        mPeriodicIO->output_voltage = 0.0;
        mPeriodicIO->master_current = 0.0;
        mPeriodicIO->error_ticks = 0.0;

    #endif

    //if (mControlState != ControlState::OPEN_LOOP)
    /*
    if (false)
    {
        mPeriodicIO->active_trajectory_position = mPeriodicIO->position_ticks;
        if (mPeriodicIO->active_trajectory_position < mReverseSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants->kName + ": Active Trajectory past reverse soft limit!");
        } else if (mPeriodicIO->active_trajectory_position > mForwardSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants->kName + ": Active Trajectory past forward soft limit!");
        } 

        int newVel = mPeriodicIO->velocity_ticks_per_100ms;
        if (util.epsilonEquals(std::round(newVel), mConstants->kMaxVelocity, std::max(1.0 , std::round(mConstants->kAllowableClosedLoopError))) || 
                util.epsilonEquals(newVel, mPeriodicIO->active_trajectory_velocity, std::max(1, (int)std::round(mConstants->kAllowableClosedLoopError))) )
        {
            mPeriodicIO->active_trajectory_acceleration = 0;
        } else 
        {
      
            mPeriodicIO->active_trajectory_acceleration = (newVel - mPeriodicIO->active_trajectory_velocity)* mConstants->kMaxAcceleration;//(mPeriodicIO->timestamp - prev_timestamp);
    //    }
        
        mPeriodicIO->active_trajectory_velocity = newVel;
            
    //} else
    //{
    */
        mPeriodicIO->active_trajectory_acceleration = 0;
        mPeriodicIO->active_trajectory_velocity = 0;
        mPeriodicIO->active_trajectory_position = std::numeric_limits<int>::max();
    //}

    // only absolute
    //if (mConstants->kRecoverPositionOnReset)
    /*
    if (false)
    {
        mPeriodicIO->absolute_position = mPeriodicIO->position_ticks;
            mPeriodicIO->absolute_position_modded = mPeriodicIO->absolute_position % mConstants->kCountsPerRev;
            if (mPeriodicIO->absolute_position_modded < 0)
            {
                mPeriodicIO->absolute_position_modded += mConstants->kCountsPerRev;
            }

        
        if (prev_position_units == NAN)
        {
            prev_position_units = mPeriodicIO->position_units;
        } else 
        {
            mPeriodicIO->absolute_position = mPeriodicIO->position_ticks;
            mPeriodicIO->absolute_position_modded = mPeriodicIO->absolute_position % mConstants->kCountsPerRev;
            if (mPeriodicIO->absolute_position_modded < 0)
            {
                mPeriodicIO->absolute_position_modded += mConstants->kCountsPerRev;
            }

            //check this is actually accurate
            double est_disp = (mPeriodicIO->timestamp - prev_timestamp) * mPeriodicIO->velocity_ticks_per_100ms;
            mPeriodicIO->encoder_wraps += (mPeriodicIO->position_ticks - (prev_position_units + est_disp)) > 2000.0? -1 : (mPeriodicIO->position_ticks - (prev_position_units + est_disp)) < -2000.0 ? 1 : 0;
        }
    } else 
    {
    */
        mPeriodicIO->encoder_wraps = 0;
        mPeriodicIO->absolute_position = 0;
        mPeriodicIO->absolute_position_modded = 0;
    //}

    prev_position_units = mPeriodicIO->position_ticks;
    prev_timestamp = mPeriodicIO->timestamp;
    
}

void SparkMaxSubsystem::writePeriodicOutputs()
{
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ Demand: ", mPeriodicIO->demand); 
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ Feedforward: ", mPeriodicIO->feedforward);
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ ControlState: ", mControlState);
    #ifdef CompetitionBot
    //std::cout << "SparkMaxSubsystem: " << mConstants->kName << ": write periodic outputs" << std::endl;
    if (mControlState == ControlState::MOTION_MAGIC)
    {
        mPIDController->SetReference(mPeriodicIO->demand, rev::ControlType::kSmartMotion, Constants::kMotionMagicPIDSlot, mPeriodicIO->feedforward);
    } else if (mControlState == ControlState::POSITION_PID)
    {
        mPIDController->SetReference(mPeriodicIO->demand, rev::ControlType::kPosition, Constants::kPositionPIDSlot, mPeriodicIO->feedforward);
    } else if (mControlState == ControlState::VELOCITY_PID)
    {
        mPIDController->SetReference(mPeriodicIO->demand, rev::ControlType::kVelocity, Constants::kVelocityPIDSlot, mPeriodicIO->feedforward);
    } else 
    {
        mPIDController->SetReference(mPeriodicIO->demand, rev::ControlType::kDutyCycle, 0, mPeriodicIO->feedforward);
    }
    #endif  
}

void SparkMaxSubsystem::zeroSensors()
{
    #ifdef CompetitionBot
    std::cout << "SparkMaxSubsystem: " << mConstants->kName << ": zeroing sensors" << std::endl;
    mPeriodicIO->absolute_offset = (int)std::round(mEncoder->GetPosition());
    mEncoder->SetPosition(0.0);
    mHasBeenZeroed = true;
    #endif
}

void SparkMaxSubsystem::OnStart(double timestamp)
{
    stop();
}

void SparkMaxSubsystem::OnLoop(double timestamp)
{

}

void SparkMaxSubsystem::OnStop(double timestamp)
{

}

void SparkMaxSubsystem::outputTelemetry()
{
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Position Units: ", mPeriodicIO->position_units);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Velocity Units: ", mPeriodicIO->velocity_units);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Percent Output: ", mPeriodicIO->output_percent);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Voltage Output: ", mPeriodicIO->output_voltage);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Current Output: ", mPeriodicIO->master_current);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Position Raw: ", mPeriodicIO->position_ticks);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Velocity Raw: ", mPeriodicIO->velocity_ticks_per_100ms);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Demand: ", mPeriodicIO->demand);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ FeedForward: ", mPeriodicIO->feedforward);
}

bool SparkMaxSubsystem::hasFinishedTrajectory()
{
    if (util.epsilonEquals(mPeriodicIO->active_trajectory_position, ticksToUnits(getSetpoint())), std::fmax(1.0, mConstants->kAllowableClosedLoopError))
    {
        return true;
    }
    return false;
}

double SparkMaxSubsystem::getActiveTrajectoryUnits()
{
    return ticksToHomedUnits(mPeriodicIO->active_trajectory_position);
}

double SparkMaxSubsystem::getActiveTrajectoryUnitsPerSecond()
{
    return ticksPer100msToUnitsPerSecond(mPeriodicIO->active_trajectory_velocity);
}

double SparkMaxSubsystem::getPredictedPositionUnits(double lookahead_secs)
{
    if (mControlState != ControlState::MOTION_MAGIC)
    {
        return getPosition();
    }

    double predicted_units = ticksToHomedUnits(mPeriodicIO->active_trajectory_position + 
        lookahead_secs * mPeriodicIO->active_trajectory_velocity + 
        .5 * mPeriodicIO->active_trajectory_acceleration * lookahead_secs * lookahead_secs);
    if (mPeriodicIO->demand >= mPeriodicIO->active_trajectory_position)
    {
        return std::fmin(predicted_units, ticksToHomedUnits(mPeriodicIO->demand));
    } else
    {
        return std::fmax(predicted_units, ticksToHomedUnits(mPeriodicIO->demand));
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
    return (mControlState != ControlState::OPEN_LOOP)? ticksToUnits(mPeriodicIO->demand) : mPeriodicIO->demand;
}

void SparkMaxSubsystem::setSetpointMotionMagic(double units, double feedforward_v)
{
    mPeriodicIO->demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO->feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants->kF.at(Constants::kMotionMagicPIDSlot) + mConstants->kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

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
    frc::SmartDashboard::PutNumber(mConstants->kName + "/setSetpointPositionPID/ pre_units", units);
    mPeriodicIO->demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO->feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants->kF.at(Constants::kMotionMagicPIDSlot) + mConstants->kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;
    frc::SmartDashboard::PutNumber(mConstants->kName + "/setSetpointPositionPID/ demand", mPeriodicIO->demand);
    frc::SmartDashboard::PutNumber(mConstants->kName + "/setSetpointPositionPID/ feedforward", mPeriodicIO->feedforward);
    
    if (mControlState != ControlState::POSITION_PID)
    {
        mControlState = ControlState::POSITION_PID;
    }
}

void SparkMaxSubsystem::setSetpointVelocityPID(double units, double feedforward_v)
{
    mPeriodicIO->demand = unitsPerSecondToTicksPer100ms(units);//constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO->feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants->kF.at(Constants::kVelocityPIDSlot) + mConstants->kD.at(Constants::kVelocityPIDSlot)/100.0)/1023.0;
    if (mControlState != ControlState::VELOCITY_PID)
    {
        mControlState = ControlState::VELOCITY_PID;
    }
}

void SparkMaxSubsystem::setOpenLoop(double percentage)
{
    mPeriodicIO->demand = percentage;
    mPeriodicIO->feedforward = 0.0;
    if (mControlState != ControlState::OPEN_LOOP)
    {
        mControlState = ControlState::OPEN_LOOP;
    }
}

double SparkMaxSubsystem::ticksToUnits(double ticks)
{
    return ticks / mConstants->kTicksPerUnitDistance;
}

double SparkMaxSubsystem::ticksToHomedUnits(double ticks)
{
    double val = ticksToUnits(ticks);
    return val + mConstants->kHomePosition;
}

double SparkMaxSubsystem::unitsToTicks(double units)
{
    return units * mConstants->kTicksPerUnitDistance;
}

double SparkMaxSubsystem::homeAwareUnitsToTicks(double units)
{
    return unitsToTicks(units - mConstants->kHomePosition);
}

double SparkMaxSubsystem::constrainTicks(double ticks)
{
    if (!(mConstants->kEnableForwardSoftLimit && mConstants->kEnableReverseSoftLimit))
    {
        return ticks;
    }
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

void SparkMaxSubsystem::pidTuning()
{
    
    double dem = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Demand: ", 0.0);
    double ff = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Feedforward: ", 0.0);
    bool running = frc::SmartDashboard::GetBoolean("Subsystems/" + mConstants->kName + "/PIDTuning/ Enabled: ", false);

    int i = mPIDMode;
    if (i < 4)
    {
    if (p != kp)
        {
            mPIDController->SetP(p, i);
            kp = p;
        } else if (i != ki)
        {
            mPIDController->SetI(i, i);
            ki = i;
        } else if (d != kd)
        {
            mPIDController->SetD(d, i);
            kd = d;
        } else if (f != kf)
        {
            mPIDController->SetFF(f, i);
            kf = f;
        }
    }

    if (running)
    {
    switch (mPIDMode)
    {
        //case MOTIONPROFILE:
        //    setGoalMotionProfiling(); // nothing yet
        //    break;
        case POSITION_PID:
            setSetpointPositionPID(dem, ff);
            break;
        case VELOCITY_PID:
            setSetpointVelocityPID(dem, ff);
            break;
        case MOTION_MAGIC:
            setSetpointMotionMagic(dem, ff);
            break;
        default:
            setOpenLoop(dem);
            break;
    }
    } else
    {
        setOpenLoop(0.0);
    }
    
    
    
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual Demand: ", dem);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual Feedforward: ", ff);
}


TalonSRXSubsystem::TalonSRXSubsystem(std::shared_ptr<TalonConstants> constants) : TalonSubsystem()
{
    #ifdef CompetitionBot
    //std::cout<<"TalonSRX Subsystem starting"<< std::endl;
    mConstants = constants;
    
    mModeChooser.SetDefaultOption("OPEN_LOOP", ControlState::OPEN_LOOP);
    mModeChooser.AddOption("MAGIC_MOTION", ControlState::MOTION_MAGIC);
    mModeChooser.AddOption("POSITION_PID", ControlState::POSITION_PID);
    mModeChooser.AddOption("VELOCITY_PID", ControlState::VELOCITY_PID);
    //mModeChooser.AddOption("MOTION_PROFILE", Modes::MOTIONPROFILE);
    frc::SmartDashboard::PutData("Subsystems/" + mConstants->kName + "/PIDTuning/Mode", &mModeChooser);
    
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Demand: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Feedforward: ", 0.0);
    frc::SmartDashboard::PutBoolean("Subsystems/" + mConstants->kName + "/PIDTuning/ Enabled: ", false);
    
    kp = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", 0.0);
    ki = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", 0.0);
    kd = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", 0.0);
    kf = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", 0.0);
    
    //std::cout<<"TalonSRX Subsystem copied constants"<< std::endl;
    std::cout <<"TalonSRX Subsystem starting: " << mConstants->id <<std::endl;
    //std::cout<<"TalonSRX Subsystem accessed Constants"<< std::endl;
    mMaster = Drivers::TalonFactory::createDefaultTalonSRX(mConstants->id);
    //std::cout<<"TalonSRX Subsystem created talon default"<< std::endl;
    
    mForwardSoftLimit = ((mConstants->kMaxUnitsLimit - mConstants->kHomePosition)*mConstants->kTicksPerUnitDistance);    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigForwardSoftLimitThreshold((int) mForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure forward soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigForwardSoftLimitEnable(mConstants->kEnableForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable forward soft limit: ");

    mReverseSoftLimit = ((mConstants->kMinUnitsLimit - mConstants->kHomePosition)*mConstants->kTicksPerUnitDistance);
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigReverseSoftLimitThreshold((int) mReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure reverse soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigReverseSoftLimitEnable(mConstants->kEnableReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable reverse soft limit: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigVoltageCompSaturation(mConstants->kVoltageCompensation, Constants::kLongCANTimeoutMs), ": could not configure Voltage Compensation: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kP(Constants::kMotionProfileSlot, mConstants->kP.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kI(Constants::kMotionProfileSlot, mConstants->kI.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kD(Constants::kMotionProfileSlot, mConstants->kD.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kF(Constants::kMotionProfileSlot, mConstants->kF.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_IntegralZone(Constants::kMotionProfileSlot, mConstants->kIZone.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMaxIntegralAccumulator(Constants::kMotionProfileSlot, mConstants->kMaxIAccum.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigAllowableClosedloopError(Constants::kMotionProfileSlot, mConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kP(Constants::kPositionPIDSlot, mConstants->kP.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kI(Constants::kPositionPIDSlot, mConstants->kI.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kD(Constants::kPositionPIDSlot, mConstants->kD.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kF(Constants::kPositionPIDSlot, mConstants->kF.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_IntegralZone(Constants::kPositionPIDSlot, mConstants->kIZone.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMaxIntegralAccumulator(Constants::kPositionPIDSlot, mConstants->kMaxIAccum.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigAllowableClosedloopError(Constants::kPositionPIDSlot, mConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kP(Constants::kVelocityPIDSlot, mConstants->kP.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kI(Constants::kVelocityPIDSlot, mConstants->kI.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kD(Constants::kVelocityPIDSlot, mConstants->kD.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kF(Constants::kVelocityPIDSlot, mConstants->kF.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_IntegralZone(Constants::kVelocityPIDSlot, mConstants->kIZone.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMaxIntegralAccumulator(Constants::kVelocityPIDSlot, mConstants->kMaxIAccum.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigAllowableClosedloopError(Constants::kVelocityPIDSlot, mConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot Deadband: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMotionCruiseVelocity(mConstants->kMaxVelocity, Constants::kLongCANTimeoutMs), ": could not configure cruise velocity: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMotionAcceleration(mConstants->kMaxAcceleration, Constants::kLongCANTimeoutMs), ": could not configure acceleration: ");
    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigClosedloopRamp(mConstants->kClosedLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure closed loop ramp rate: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigOpenloopRamp(mConstants->kOpenLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure open loop ramp rate: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigContinuousCurrentLimit(mConstants->kContinuousCurrentLimit, Constants::kLongCANTimeoutMs), ": could not configure continuous current limit: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigPeakCurrentLimit(mConstants->kPeakCurrentLimit, Constants::kLongCANTimeoutMs), ": could not configure peak current limit: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigPeakCurrentDuration(mConstants->kPeakCurrentDuration, Constants::kLongCANTimeoutMs), ": could not configure peak current duration: ");

    
    mMaster->EnableVoltageCompensation(mConstants->kEnableVoltageCompensation);
    mMaster->SetInverted(mConstants->inverted);
    mMaster->SetSensorPhase(mConstants->kInvertSensorPhase);
    mMaster->SetNeutralMode(mConstants->kNeutralMode);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants->kStatusFrame8UpdateRate, 20);
    
    //std::cout<<"ServoMotorSubsystem-TalonSRXSubsystem(): Starting Slaves"<<std::endl;
    if (mConstants->kSlaveIDs.size() == 0){
        //std::cout<<"ServoMotorSubsystem-TalonSRXSubsystem(): Skipping Slaves"<<std::endl;
    } else
    {
        
    for (auto slave: mConstants->kSlaveIDs)
    {
        std::cout<< "Creating Slave TalonSRX: " << slave->id <<slave->invert_motor << slave->isTalonSRX << std::endl; 
        if ((mConstants->kSlaveIDs.at(i)->id) == -1)
        {
            continue;
        }
        if (slave->isTalonSRX)
        {
            mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonSRX(slave->id, mMaster));
        
        } else 
        {
            mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonFX(slave->id, mMaster));
        }
        //std::cout<<"ServoMotorSubsystem-TalonSRXSubsystem(): added Slaves"<<std::endl;
        mSlaves.at(i)->SetInverted(mConstants->kSlaveIDs.at(i)->invert_motor);
        mSlaves.at(i)->SetNeutralMode(mConstants->kNeutralMode);
        i++;
    }    
    }
    
    
    //std::cout<<"ServoMotorSubsystem-TalonSRXSubsystem(): finished Slaves"<<std::endl;
    #endif
}
 
void TalonSRXSubsystem::readPeriodicInputs()
{
    #ifdef CompetitionBot
    if (mConstants->readInputs)
    {
    //std::cout << "TalonSRXSubsystem: " << mConstants->kName << ": read periodic inputs" << std::endl;
    mPeriodicIO->timestamp = frc::Timer::GetFPGATimestamp();

    if ( mMaster->HasResetOccurred() ) 
    {
        frc::DriverStation::ReportError(mConstants->kName + ": Talon Reset! ");
        mPeriodicIO->reset_occurred = true;
        return;
    } else
    {
        mPeriodicIO->reset_occurred = false;
    }

    mMaster->GetStickyFaults(mFaults);
    if (mFaults.HasAnyFault())
    {
        frc::DriverStation::ReportError(mConstants->kName + ": Talon Fault! " + mFaults.ToString());
        mMaster->ClearStickyFaults();
    }

    if (mMaster->GetControlMode() == ControlMode::MotionMagic)
    {
        mPeriodicIO->active_trajectory_position = mMaster->GetActiveTrajectoryPosition();

        if (mPeriodicIO->active_trajectory_position < mReverseSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants->kName + ": Active Trajectory Past Reverse soft limit!");
        } else 
        {
            frc::DriverStation::ReportError(mConstants->kName + ": Active Trajectory Past Forward soft limit!");
        }
        int newVel = mMaster->GetActiveTrajectoryVelocity();
        if (util.epsilonEquals(newVel, mConstants->kMaxVelocity, std::fmax(1.0, mConstants->kAllowableClosedLoopError)) || 
            util.epsilonEquals(newVel, mPeriodicIO->active_trajectory_velocity, std::fmax(1.0, mConstants->kAllowableClosedLoopError)))
        {
            //Mechanism is at a constant velocity (velocity is at max or didn't change)
            mPeriodicIO->active_trajectory_acceleration = 0.0;
        } else
        {
            mPeriodicIO->active_trajectory_acceleration = (newVel - mPeriodicIO->active_trajectory_velocity) * mConstants->kMaxAcceleration;
        }
        mPeriodicIO->active_trajectory_velocity = newVel;
    } else
    {
        mPeriodicIO->active_trajectory_position = 0;
        mPeriodicIO->active_trajectory_velocity = 0;
        mPeriodicIO->active_trajectory_acceleration = 0;
    }
    if (mMaster->GetControlMode() == ControlMode::Position)
    {
        mPeriodicIO->error_ticks = mMaster->GetClosedLoopError();
    } else
    {
        mPeriodicIO->error_ticks = 0;
    }
    mPeriodicIO->master_current = mMaster->GetOutputCurrent();
    mPeriodicIO->output_voltage = mMaster->GetMotorOutputVoltage();
    mPeriodicIO->output_percent = mMaster->GetMotorOutputPercent();
    mPeriodicIO->position_ticks = mMaster->GetSelectedSensorPosition();
    mPeriodicIO->position_units = ticksToHomedUnits(mPeriodicIO->position_ticks);
    mPeriodicIO->velocity_ticks_per_100ms = mMaster->GetSelectedSensorVelocity();
    mPeriodicIO->velocity_units = ticksPer100msToUnitsPerSecond(mPeriodicIO->velocity_ticks_per_100ms);
    
    if (mConstants->kRecoverPositionOnReset)
    {
        mPeriodicIO->absolute_position = mMaster->GetSensorCollection().GetPulseWidthPosition();
        mPeriodicIO->absolute_position_modded = mPeriodicIO->absolute_position % 4096;
        if (mPeriodicIO->absolute_position_modded < 0)
        {
            mPeriodicIO->absolute_position_modded += 4096;
        }

        int estimated_pulsed_pos = ((mConstants->invert_sensor_phase ? -1 : 1)*mPeriodicIO->position_ticks) + mPeriodicIO->absolute_offset;
        int new_wraps = (int) std::round(estimated_pulsed_pos/4096.0);

        if (std::abs(mPeriodicIO->encoder_wraps - new_wraps) <= 1 )
        {
            mPeriodicIO->encoder_wraps = new_wraps;
        }
    } else 
    {
        mPeriodicIO->absolute_position = 0.0;
        mPeriodicIO->absolute_position_modded = 0.0;
    }
    }
    
    mPIDMode = mModeChooser.GetSelected();
    k++;
    int i = mControlState;
    if (i < 4){
    
    p = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", mConstants->kP.at(i));
    i = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", mConstants->kI.at(i));
    d = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", mConstants->kD.at(i));
    f = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", mConstants->kF.at(i));
    

    
    if (k % 10 == 0)
    {
        frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kP: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_P, i, 0));
        frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kI: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_I, i, 0));
        frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kD: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_D, i, 0));
        frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kF: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_F, i, 0));
            
    }
    }
    #endif
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual mode: ", i);
}

 
void TalonSRXSubsystem::writePeriodicOutputs()
{

    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ Demand: ", mPeriodicIO->demand);
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ Feedforward: ", mPeriodicIO->feedforward);
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ ControlState: ", mControlState);
    #ifdef CompetitionBot
    //std::cout << "TalonSRXSubsystem: " << mConstants->kName << ": write periodic outputs" << std::endl;
    if (mControlState == ControlState::MOTION_MAGIC)
    {
        mMaster->Set(ControlMode::MotionMagic, mPeriodicIO->demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO->feedforward);
    } else if (mControlState == ControlState::POSITION_PID || mControlState == ControlState::MOTION_PROFILING)
    {
        mMaster->Set(ControlMode::Position, mPeriodicIO->demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO->feedforward);
    } else if (mControlState == ControlState::VELOCITY_PID)
    {
        mMaster->Set(ControlMode::Velocity, mPeriodicIO->demand);
    } else
    {
        mMaster->Set(ControlMode::PercentOutput, mPeriodicIO->demand);
    }
    #endif
}

 
void TalonSRXSubsystem::zeroSensors()
{
    #ifdef CompetitionBot
    //std::cout << "TalonSRXSubsystem: " << mConstants->kName << ": zeroing sensors" << std::endl;
    mMaster->SetSelectedSensorPosition(0.0, Constants::kCANTimeoutMs);
    if (mConstants->kIsTalonSRX)
    {
        mPeriodicIO->absolute_offset = getAbsoluteEncoderRawPosition(mMaster->GetSensorCollection().GetPulseWidthPosition());
    }
    mHasBeenZeroed = true;
    #endif
}

 
void TalonSRXSubsystem::OnStart(double timestamp)
{
    //std::cout << "TalonSRXSubsystem: " << mConstants->kName << ": OnStart" << std::endl;
    stop();
}

 
void TalonSRXSubsystem::OnLoop(double timestamp)
{
    #ifdef CompetitionBot
    //std::cout << "TalonSRXSubsystem: " << mConstants->kName << ": OnLoop" << std::endl;
    if (mPeriodicIO->reset_occurred)
    {
        std::cout << mConstants->kName << " : Master Talon reset occurred; resetting frame rates."<<std::endl;
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants->kStatusFrame8UpdateRate, 20);
    
        if (mConstants->kRecoverPositionOnReset)
        {
            mMaster->SetSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants::kCANTimeoutMs);
        }
    }
    handleMasterReset(mPeriodicIO->reset_occurred);
    if (mSlaves.size() == 0)
    {

    } else
    {
        for (auto slave : mSlaves)
        {
            if (slave->HasResetOccurred())
            {
                std::cout << mConstants->kName << " : Slave Talon reset occurred."<<std::endl;
            }
        }
    }
    #endif
}


void TalonSRXSubsystem::OnStop(double timestamp)
{
    #ifdef CompetitionBot
    //std::cout << "TalonSRXSubsystem: " << mConstants->kName << ": OnStop" << std::endl;
    stop();
    #endif
}

void TalonSubsystem::outputTelemetry()
{
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Position Units: ", mPeriodicIO->position_units);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Velocity Units: ", mPeriodicIO->velocity_units);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Percent Output: ", mPeriodicIO->output_percent);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Voltage Output: ", mPeriodicIO->output_voltage);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Current Output: ", mPeriodicIO->master_current);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Position Raw: ", mPeriodicIO->position_ticks);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Velocity Raw: ", mPeriodicIO->velocity_ticks_per_100ms);
    
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ Demand: ", mPeriodicIO->demand);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/ FeedForward: ", mPeriodicIO->feedforward);
}


bool TalonSubsystem::hasFinishedTrajectory()
{
    if (util.epsilonEquals((double) mPeriodicIO->active_trajectory_position, ticksToUnits(getSetpoint()), std::fmax(1.0, mConstants->kAllowableClosedLoopError)))
    {
        return true;
    }
    return false;
}


double TalonSubsystem::getActiveTrajectoryUnits()
{
    return ticksToHomedUnits(mPeriodicIO->active_trajectory_position);
}


double TalonSubsystem::getActiveTrajectoryUnitsPerSecond()
{
    return ticksPer100msToUnitsPerSecond(mPeriodicIO->active_trajectory_velocity);
}


double TalonSubsystem::getPredictedPositionUnits(double lookahead_secs)
{
    if (mControlState != ControlState::MOTION_MAGIC)
    {
        return getPosition();
    }

    double predicted_units = ticksToHomedUnits(mPeriodicIO->active_trajectory_position +
            lookahead_secs * mPeriodicIO->active_trajectory_velocity +
            mPeriodicIO->active_trajectory_acceleration * lookahead_secs * lookahead_secs);
    if(mPeriodicIO->demand >= mPeriodicIO->active_trajectory_position)
    {
        return std::fmin(predicted_units, ticksToHomedUnits(mPeriodicIO->demand));
    } else
    {
        return std::fmax(predicted_units, ticksToHomedUnits(mPeriodicIO->demand));
    }
}


bool TalonSubsystem::atHomingLocation()
{
    return false;
}


void TalonSubsystem::resetIfAtLimit()
{
    if (atHomingLocation())
    {
        zeroSensors();
    }
}


int TalonSubsystem::getAbsoluteEncoderRawPosition(int pulse_width_position)
{
    int abs_raw_with_rollover = pulse_width_position % 4096;
    return abs_raw_with_rollover + (abs_raw_with_rollover < 0 ? abs_raw_with_rollover + 4096 : 0);
}


bool TalonSubsystem::hasBeenZeroed()
{
    return mHasBeenZeroed;
}


void TalonSubsystem::stop()
{
    setOpenLoop(0.0);
}


int TalonSubsystem::estimateSensorPositionFromAbsolute()
{
    int estimated_pulse_pos = (mPeriodicIO->encoder_wraps * 4096) + mPeriodicIO->absolute_position_modded;
    int estimate_position_ticks = (mConstants->invert_sensor_phase ? -1 : 1) * (estimated_pulse_pos -mPeriodicIO->absolute_offset);
    return estimate_position_ticks;
}


double TalonSubsystem::getSetpoint()
{
    return (mControlState != ControlState::OPEN_LOOP) ? ticksToUnits(mPeriodicIO->demand) : mPeriodicIO->demand;
}


void TalonSubsystem::setSetpointMotionMagic(double units, double feedforward_v)
{
    mPeriodicIO->demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO->feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants->kF.at(Constants::kMotionMagicPIDSlot) + mConstants->kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::MOTION_MAGIC)
    {
        mControlState = ControlState::MOTION_MAGIC;
    }
}


void TalonSubsystem::setSetpointMotionMagic(double units)
{
    setSetpointMotionMagic(units, 0.0);
}


void TalonSubsystem::setSetpointPositionPID(double units, double feedforward_v)
{
    mPeriodicIO->demand = constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO->feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants->kF.at(Constants::kMotionMagicPIDSlot) + mConstants->kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::POSITION_PID)
    {
        mControlState = ControlState::POSITION_PID;
    }
}


void TalonSubsystem::setSetpointVelocityPID(double units, double feedforward_v)
{
    mPeriodicIO->demand = unitsPerSecondToTicksPer100ms(units);//constrainTicks(homeAwareUnitsToTicks(units));
    //check feedforward...
    mPeriodicIO->feedforward = unitsPerSecondToTicksPer100ms(feedforward_v) * (mConstants->kF.at(Constants::kMotionMagicPIDSlot) + mConstants->kD.at(Constants::kMotionMagicPIDSlot)/100.0)/1023.0;

    if (mControlState != ControlState::VELOCITY_PID)
    {
        mControlState = ControlState::VELOCITY_PID;
    }
}


void TalonSubsystem::setOpenLoop(double percentage)
{
    mPeriodicIO->demand = percentage;
    if (mControlState != ControlState::OPEN_LOOP)
    {
        mControlState = ControlState::OPEN_LOOP;
    }
}


double TalonSubsystem::ticksToUnits(double ticks)
{
    return ticks/mConstants->kTicksPerUnitDistance;
}


double TalonSubsystem::ticksToHomedUnits(double ticks)
{
    double val = ticksToUnits(ticks);
    return val + mConstants->kHomePosition;
}


double TalonSubsystem::unitsToTicks(double units)
{
    return units * mConstants->kTicksPerUnitDistance;
}


double TalonSubsystem::homeAwareUnitsToTicks(double units)
{
    return unitsToTicks(units-mConstants->kHomePosition);
}


double TalonSubsystem::constrainTicks(double ticks)
{
    if (!(mConstants->kEnableForwardSoftLimit && mConstants->kEnableReverseSoftLimit))
    {
        return ticks;
    }
    return util.limit(ticks, mReverseSoftLimit, mForwardSoftLimit);
}


double TalonSubsystem::ticksPer100msToUnitsPerSecond(double ticks_per_100ms)
{
    return ticksToUnits(ticks_per_100ms) * 10.0;
}


double TalonSubsystem::unitsPerSecondToTicksPer100ms(double units_per_second)
{
    return unitsToTicks(units_per_second) / 10.0;
}

void TalonSRXSubsystem::pidTuning()
{
    
    double dem = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Demand: ", 0.0);
    double ff = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Feedforward: ", 0.0);
    bool running = frc::SmartDashboard::GetBoolean("Subsystems/" + mConstants->kName + "/PIDTuning/ Enabled: ", false);    

    int i = mPIDMode;
    if (i < 4)
    {
        
    if (p != kp)
    {
        mMaster->Config_kP(i, p);
        kp = p;
    } else if (i != ki)
    {
        mMaster->Config_kI(i, i);
        ki = i;
    } else if (d != kd)
    {
        mMaster->Config_kD(i, d);
        kd = d;
    } else if (f != kf)
    {
        mMaster->Config_kF(i, f);
        kf = f;
    }
    }
    

    if (running)
    {
    switch (mPIDMode)
    {
        
        case POSITION_PID:
            setSetpointPositionPID(dem, ff);
            break;
        case VELOCITY_PID:
            setSetpointVelocityPID(dem, ff);
            break;
        case MOTION_MAGIC:
            setSetpointMotionMagic(dem, ff);
            break;
        default:
            setOpenLoop(dem);
            break;
    }
    } else
    {
        setOpenLoop(0.0);
    }
    

    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual Demand: ", dem);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual Feedforward: ", ff);
}

TalonFXSubsystem::TalonFXSubsystem(std::shared_ptr<TalonConstants> constants) : TalonSubsystem() 
{
    #ifdef CompetitionBot
    mConstants = constants;
    
    mModeChooser.SetDefaultOption("OPEN_LOOP", ControlState::OPEN_LOOP);
    mModeChooser.AddOption("MAGIC_MOTION", ControlState::MOTION_MAGIC);
    mModeChooser.AddOption("POSITION_PID", ControlState::POSITION_PID);
    mModeChooser.AddOption("VELOCITY_PID", ControlState::VELOCITY_PID);
    //mModeChooser.AddOption("MOTION_PROFILE", Modes::MOTIONPROFILE);
    frc::SmartDashboard::PutData("Subsystems/" + mConstants->kName + "/PIDTuning/Mode", &mModeChooser);
    
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Demand: ", 0.0);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Feedforward: ", 0.0);
    frc::SmartDashboard::PutBoolean("Subsystems/" + mConstants->kName + "/PIDTuning/ Enabled: ", false);
    
    kp = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", 0.0);
    ki = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", 0.0);
    kd = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", 0.0);
    kf = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", 0.0);

    std::cout << "Starting TalonFX Subsystem: " << mConstants->id << std::endl;
    mMaster = Drivers::TalonFactory::createDefaultTalonFX(mConstants->id);

    mForwardSoftLimit = ((mConstants->kMaxUnitsLimit - mConstants->kHomePosition)*mConstants->kTicksPerUnitDistance);    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigForwardSoftLimitThreshold((int) mForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure forward soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigForwardSoftLimitEnable(mConstants->kEnableForwardSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable forward soft limit: ");

    mReverseSoftLimit = ((mConstants->kMinUnitsLimit - mConstants->kHomePosition)*mConstants->kTicksPerUnitDistance);
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigReverseSoftLimitThreshold((int) mReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not configure reverse soft limit: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigReverseSoftLimitEnable(mConstants->kEnableReverseSoftLimit, Constants::kLongCANTimeoutMs), ": could not enable reverse soft limit: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigVoltageCompSaturation(mConstants->kVoltageCompensation, Constants::kLongCANTimeoutMs), ": could not configure Voltage Compensation: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kP(Constants::kMotionProfileSlot, mConstants->kP.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kI(Constants::kMotionProfileSlot, mConstants->kI.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kD(Constants::kMotionProfileSlot, mConstants->kD.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kF(Constants::kMotionProfileSlot, mConstants->kF.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_IntegralZone(Constants::kMotionProfileSlot, mConstants->kIZone.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMaxIntegralAccumulator(Constants::kMotionProfileSlot, mConstants->kMaxIAccum.at(Constants::kMotionProfileSlot), Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigAllowableClosedloopError(Constants::kMotionProfileSlot, mConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kMotionProfileSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kP(Constants::kPositionPIDSlot, mConstants->kP.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kI(Constants::kPositionPIDSlot, mConstants->kI.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kD(Constants::kPositionPIDSlot, mConstants->kD.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kF(Constants::kPositionPIDSlot, mConstants->kF.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_IntegralZone(Constants::kPositionPIDSlot, mConstants->kIZone.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMaxIntegralAccumulator(Constants::kPositionPIDSlot, mConstants->kMaxIAccum.at(Constants::kPositionPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigAllowableClosedloopError(Constants::kPositionPIDSlot, mConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kPositionPIDSlot Deadband: ");
    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kP(Constants::kVelocityPIDSlot, mConstants->kP.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kP: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kI(Constants::kVelocityPIDSlot, mConstants->kI.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kI: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kD(Constants::kVelocityPIDSlot, mConstants->kD.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kD: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_kF(Constants::kVelocityPIDSlot, mConstants->kF.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kF: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->Config_IntegralZone(Constants::kVelocityPIDSlot, mConstants->kIZone.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kIZone: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMaxIntegralAccumulator(Constants::kVelocityPIDSlot, mConstants->kMaxIAccum.at(Constants::kVelocityPIDSlot), Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot kMaxIAccum: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigAllowableClosedloopError(Constants::kVelocityPIDSlot, mConstants->kAllowableClosedLoopError, Constants::kLongCANTimeoutMs), ": could not configure kVelocityPIDSlot Deadband: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMotionCruiseVelocity(mConstants->kMaxVelocity, Constants::kLongCANTimeoutMs), ": could not configure cruise velocity: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigMotionAcceleration(mConstants->kMaxAcceleration, Constants::kLongCANTimeoutMs), ": could not configure acceleration: ");
    
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigClosedloopRamp(mConstants->kClosedLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure closed loop ramp rate: ");
    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigOpenloopRamp(mConstants->kOpenLoopRampRate, Constants::kLongCANTimeoutMs), ": could not configure open loop ramp rate: ");

    Drivers::TalonFactory::handleCANError(mConstants->id, mMaster->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, (double) mConstants->kPeakCurrentLimit, (double) mConstants->kContinuousCurrentLimit, (double) mConstants->kPeakCurrentDuration}, Constants::kLongCANTimeoutMs), ": could not configure Supply Current Limit: ");
    
    mMaster->EnableVoltageCompensation(mConstants->kEnableVoltageCompensation);
    mMaster->SetInverted(mConstants->inverted);
    mMaster->SetSensorPhase(mConstants->kInvertSensorPhase);
    mMaster->SetNeutralMode(mConstants->kNeutralMode);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
    mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants->kStatusFrame8UpdateRate, 20);

    if (mConstants->kSlaveIDs.size() == 0)
    {
        std::cout << "TalonFXSubsystem: Skipping Slaves" << std::endl;   
    } else
    {
        
    for (auto slave : mConstants->kSlaveIDs)
    {
        std::cout<< "Creating Slave TalonFX: " << slave->id <<slave->invert_motor << slave->isTalonSRX << std::endl;
        if ((slave->id) == -1)
        {
            std::cout << "Not Creating TalonFX Slave: " << slave->id << "for :" << mConstants->id << std::endl;
            continue;
        }
        if (slave->isTalonSRX)
        {
            mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonSRX(slave->id, mMaster));
        } else 
        {
            mSlaves.push_back(Drivers::TalonFactory::createSlaveTalonFX(slave->id, mMaster));
        }
        mSlaves.at(i)->SetInverted(slave->invert_motor);
        mSlaves.at(i)->SetNeutralMode(mConstants->kNeutralMode);
        i++;
        
    }    
    }
    #endif
}
 
void TalonFXSubsystem::readPeriodicInputs()
{
    #ifdef CompetitionBot
    if (mConstants->readInputs)
    {
    //std::cout << "TalonFXSubsystem: " << mConstants->kName << ": read periodic inputs" << std::endl;
    mPeriodicIO->timestamp = frc::Timer::GetFPGATimestamp();

    if ( mMaster->HasResetOccurred() ) 
    {
        frc::DriverStation::ReportError(mConstants->kName + ": Talon Reset! ");
        mPeriodicIO->reset_occurred = true;
        return;
    } else
    {
        mPeriodicIO->reset_occurred = false;
    }

    mMaster->GetStickyFaults(mFaults);
    if (mFaults.HasAnyFault())
    {
        frc::DriverStation::ReportError(mConstants->kName + ": Talon Fault! " + mFaults.ToString());
        mMaster->ClearStickyFaults();
    }

    if (mMaster->GetControlMode() == ControlMode::MotionMagic)
    {
        mPeriodicIO->active_trajectory_position = mMaster->GetActiveTrajectoryPosition();

        if (mPeriodicIO->active_trajectory_position < mReverseSoftLimit)
        {
            frc::DriverStation::ReportError(mConstants->kName + ": Active Trajectory Past Reverse soft limit!");
        } else 
        {
            frc::DriverStation::ReportError(mConstants->kName + ": Active Trajectory Past Forward soft limit!");
        }
        int newVel = mMaster->GetActiveTrajectoryVelocity();
        if (util.epsilonEquals(newVel, mConstants->kMaxVelocity, std::fmax(1.0, mConstants->kAllowableClosedLoopError)) || 
            util.epsilonEquals(newVel, mPeriodicIO->active_trajectory_velocity, std::fmax(1.0, mConstants->kAllowableClosedLoopError)))
        {
            //Mechanism is at a constant velocity (velocity is at max or didn't change)
            mPeriodicIO->active_trajectory_acceleration = 0.0;
        } else
        {
            mPeriodicIO->active_trajectory_acceleration = (newVel - mPeriodicIO->active_trajectory_velocity) * mConstants->kMaxAcceleration;
        }
        mPeriodicIO->active_trajectory_velocity = newVel;
    } else
    {
        mPeriodicIO->active_trajectory_position = 0;
        mPeriodicIO->active_trajectory_velocity = 0;
        mPeriodicIO->active_trajectory_acceleration = 0;
    }
    if (mMaster->GetControlMode() == ControlMode::Position)
    {
        mPeriodicIO->error_ticks = mMaster->GetClosedLoopError();
    } else
    {
        mPeriodicIO->error_ticks = 0;
    }
    mPeriodicIO->master_current = mMaster->GetOutputCurrent();
    mPeriodicIO->output_voltage = mMaster->GetMotorOutputVoltage();
    mPeriodicIO->output_percent = mMaster->GetMotorOutputPercent();
    mPeriodicIO->position_ticks = mMaster->GetSelectedSensorPosition();
    mPeriodicIO->position_units = ticksToHomedUnits(mPeriodicIO->position_ticks);
    mPeriodicIO->velocity_ticks_per_100ms = mMaster->GetSelectedSensorVelocity();
    mPeriodicIO->velocity_units = ticksPer100msToUnitsPerSecond(mPeriodicIO->velocity_ticks_per_100ms);
    
    if (mConstants->kRecoverPositionOnReset && mConstants->kIsTalonSRX)
    {
        mPeriodicIO->absolute_position = 0.0; //mMaster->GetSensorCollection().GetPulseWidthPosition();
        mPeriodicIO->absolute_position_modded = mPeriodicIO->absolute_position % 4096;
        if (mPeriodicIO->absolute_position_modded < 0)
        {
            mPeriodicIO->absolute_position_modded += 4096;
        }

        int estimated_pulsed_pos = ((mConstants->invert_sensor_phase ? -1 : 1)*mPeriodicIO->position_ticks) + mPeriodicIO->absolute_offset;
        int new_wraps = (int) std::round(estimated_pulsed_pos/4096.0);

        if (std::abs(mPeriodicIO->encoder_wraps - new_wraps) <= 1 )
        {
            mPeriodicIO->encoder_wraps = new_wraps;
        }
    } else 
    {
        mPeriodicIO->absolute_position = 0.0;
        mPeriodicIO->absolute_position_modded = 0.0;
    }
    }
    

    mPIDMode = mModeChooser.GetSelected();
    k++;
    int i = mControlState;
    if (i < 4){
    
        p = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kP: ", mConstants->kP.at(i));
        i = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kI: ", mConstants->kI.at(i));
        d = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kD: ", mConstants->kD.at(i));
        f = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ kF: ", mConstants->kF.at(i));
    
        if (k % 10 == 0)
        {
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kP: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_P, i, 0) );
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kI: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_I, i, 0));
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kD: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_D, i, 0));
            frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual kF: ", mMaster->ConfigGetParameter(ParamEnum::eProfileParamSlot_F, i, 0));
            
        }
    }
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual mode: ", i);
    #endif
}

 
void TalonFXSubsystem::writePeriodicOutputs()
{
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ Demand: ", mPeriodicIO->demand);
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ Feedforward: ", mPeriodicIO->feedforward);
    frc::SmartDashboard::PutNumber(mConstants->kName + "/WritePeriodicOutputs/ ControlState: ", mControlState);
    #ifdef CompetitionBot
    //std::cout << "TalonFXSubsystem: " << mConstants->kName << ": write periodic outputs" << std::endl;
    if (mControlState == ControlState::MOTION_MAGIC)
    {
        mMaster->Set(ControlMode::MotionMagic, mPeriodicIO->demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO->feedforward);
    } else if (mControlState == ControlState::POSITION_PID || mControlState == ControlState::MOTION_PROFILING)
    {
        mMaster->Set(ControlMode::Position, mPeriodicIO->demand, DemandType::DemandType_ArbitraryFeedForward, mPeriodicIO->feedforward);
    } else if (mControlState == ControlState::VELOCITY_PID)
    {
        mMaster->Set(TalonFXControlMode::Velocity, 1.5*mPeriodicIO->demand);
    } else
    {
        mMaster->Set(ControlMode::PercentOutput, mPeriodicIO->demand);
    }
    #endif
}

 
void TalonFXSubsystem::zeroSensors()
{
    #ifdef CompetitionBot
    //std::cout << "TalonFXSubsystem: " << mConstants->kName << ": zero sensors" << std::endl;
    mMaster->SetSelectedSensorPosition(0.0, Constants::kCANTimeoutMs);
    if (mConstants->kIsTalonSRX)
    {
        mPeriodicIO->absolute_offset = 0.0;//getAbsoluteEncoderRawPosition(mMaster->GetSensorCollection().GetPulseWidthPosition());
    }
    mHasBeenZeroed = true;
    #endif
}

 
void TalonFXSubsystem::OnStart(double timestamp)
{
    //std::cout << "TalonFXSubsystem: " << mConstants->kName << ": OnStart" << std::endl;
    stop();
}

 
void TalonFXSubsystem::OnLoop(double timestamp)
{
    #ifdef CompetitionBot
    //std::cout << "TalonFXSubsystem: " << mConstants->kName << ": OnLoop" << std::endl;
    if (mPeriodicIO->reset_occurred)
    {
        std::cout << mConstants->kName << " : Master Talon reset occurred; resetting frame rates."<<std::endl;
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 20);
        mMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, mConstants->kStatusFrame8UpdateRate, 20);
    
        if (mConstants->kRecoverPositionOnReset)
        {
            mMaster->SetSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants::kCANTimeoutMs);
        }
    }
    handleMasterReset(mPeriodicIO->reset_occurred);
    for (auto slave : mSlaves)
    {
        if (slave->HasResetOccurred())
        {
            std::cout << mConstants->kName << " : Slave Talon reset occurred."<<std::endl;
        }
    }
    #endif
}


void TalonFXSubsystem::OnStop(double timestamp)
{
    #ifdef CompetitionBot
    //std::cout << "TalonFXSubsystem: " << mConstants->kName << ": OnStop" << std::endl;
    stop();
    #endif
}

void TalonFXSubsystem::pidTuning()
{
    
    double dem = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Demand: ", 0.0);
    double ff = frc::SmartDashboard::GetNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ PID Feedforward: ", 0.0);
    bool running = frc::SmartDashboard::GetBoolean("Subsystems/" + mConstants->kName + "/PIDTuning/ Enabled: ", false);

    int i = mPIDMode;
    if ( i < 4)
    {
        if (p != kp)
        {
            mMaster->Config_kP(i, p);
            kp = p;
        } else if (i != ki)
        {
            mMaster->Config_kI(i, i);
            ki = i;
        } else if (d != kd)
        {
            mMaster->Config_kD(i, d);
            kd = d;
        } else if (f != kf)
        {
            mMaster->Config_kF(i, f);
            kf = f;
        }
    }

    if (running)
    {
    switch (mPIDMode)
    {
        //case MOTIONPROFILE:
        //    setGoalMotionProfiling(); // nothing yet
        //    break;
        case POSITION_PID:
            setSetpointPositionPID(dem, ff);
            break;
        case VELOCITY_PID:
            setSetpointVelocityPID(dem, ff);
            break;
        case MOTION_MAGIC:
            setSetpointMotionMagic(dem, ff);
            break;
        default:
            setOpenLoop(dem);
            break;
    }
    } else
    {
        setOpenLoop(0.0);
    }
    
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual Demand: ", dem);
    frc::SmartDashboard::PutNumber("Subsystems/" + mConstants->kName + "/PIDTuning/ Actual Feedforward: ", ff);
}