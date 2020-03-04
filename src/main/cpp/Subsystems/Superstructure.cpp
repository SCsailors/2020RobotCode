/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Superstructure.h"
#include "Subsystems/LimelightManager.h"
#include "Constants.h"
using namespace Subsystems;
std::shared_ptr<Subsystems::Superstructure> Superstructure::mInstance;

Superstructure::Superstructure() 
{
    mBallPathTop = Subsystems::BallPathTop::getInstance();
    mCenteringIntake = Subsystems::CenteringIntake::getInstance();
    mHood = Subsystems::Hood::getInstance();
    mShooter = Subsystems::Shooter::getInstance();
    mTurret = Subsystems::Turret::getInstance();
    mRobotState = FRC_7054::RobotState::getInstance();

    frc::SmartDashboard::PutNumber("Subsystems /" + mTurret->mConstants->kName + "/ Turret Offset degrees", 0.0);
}

std::shared_ptr<Subsystems::Superstructure> Superstructure::getInstance()
{
    if(!mInstance)
    {
        mInstance = std::make_shared<Subsystems::Superstructure>();
    }
    return mInstance;
}

void Superstructure::OnStart(double timestamp){}

void Superstructure::OnLoop(double timestamp)
{
    mStateMachine.updatePreBottomPathState(mBallPathTop->getPreBottomState());
    mStateMachine.updateTopPathState(mBallPathTop->getFirstBreakState());
    mStateMachine.updateBottomPathState(mBallPathTop->getLastBreakState());
    
    updateCurrentState(); //gets data from subsystems
    if (!frc::SmartDashboard::GetBoolean("Enable PID Tuning", false))
    {
        //calculate intake, hood, and shooter setpoints and set it to goal
        setGoal(mStateMachine.mergedShootingIntaking(timestamp, mWantedActionShooter, mWantedActionIntake, mCurrentState, mLatestAimingParameters.getRange()));
        updateWantedAction(); //update wanted action because state transitions in superstructure state machine could change it
        
        //update turret 
        maybeUpdateGoalFromVision(timestamp);
        maybeUpdateGoalFromFieldRelativeGoal(timestamp);
        //set updated data
        followSetpoint();
    }
}

void Superstructure::OnStop(double timestamp)
{
    mWantedActionIntake = StateMachines::SuperstructureStateMachine::WantedAction::WANTED_IDLE;
    mWantedActionShooter = StateMachines::SuperstructureStateMachine::WantedAction::WANTED_IDLE;
}

SuperstructureGoal Superstructure::getGoal()
{
    return mGoal;
}

SuperstructureState Superstructure::getCurrentState()
{
    return mCurrentState;
}

SuperstructureGoal Superstructure::getSetpoint()
{
    return mCurrentSetpoint;
}

void Superstructure::jogTurret(double delta)
{
    mTurretMode = TurretControlModes::JOGGING;
    frc::SmartDashboard::PutNumber("Turret Jog Delta", delta);
    mGoal.state.turret = mCurrentState.turret + delta;
    mTurretFeedforwardV = 0.0;
}

void Superstructure::setGoal(SuperstructureGoal goal)
{
    if (mTurretMode == TurretControlModes::VISION_AIMED && mHasTarget)
    {
        //keep existing turret setpoint.
    } 
    /*
    else
    {
        mGoal.state.turret = goal.state.turret;
    }
    */
    mGoal.state.hood = goal.state.hood;
    mGoal.state.shooter = goal.state.shooter;
    mGoal.state.centeringIntake = goal.state.centeringIntake;
    mGoal.state.ballPathTop = goal.state.ballPathTop;
    mGoal.state.numBalls = goal.state.numBalls; //should just be passed through
    mGoal.state.extendIntake = goal.state.extendIntake;
    mGoal.state.extendWheelieBar = goal.state.extendWheelieBar;
}

void Superstructure::maybeUpdateGoalFromFieldRelativeGoal(double timestamp)
{
    if (mTurretMode != TurretControlModes::FIELD_RELATIVE && mTurretMode != TurretControlModes::VISION_AIMED)
    {
        mFieldRelativeGoal = NULL;
        return;
    }
    if (mTurretMode == TurretControlModes::VISION_AIMED && mHasTarget)
    {
        //vision controls turret;
        return;
    }
    if(mFieldRelativeGoal == NULL)
    {
        mTurretMode = TurretControlModes::ROBOT_RELATIVE;
        return;
    }

    double kLookaheadTime = .7;
    std::shared_ptr<Rotation2D> turret_error = mRobotState->getPredictedFieldToVehicle(kLookaheadTime)
            ->transformBy(mRobotState->getVehicleToTurret(timestamp))->getRotation()->inverse()
            ->rotateBy(mFieldRelativeGoal);
    mGoal.state.turret = mCurrentState.turret + turret_error->getDegrees();

    if (mGoal.state.turret < mTurret->getMinUnits())
    {
        mGoal.state.turret = mTurret->getMinUnits();
    } 
    if (mGoal.state.turret > mTurret->getMaxUnits())
    {
        mGoal.state.turret = mTurret->getMaxUnits();
    }

}

void Superstructure::maybeUpdateGoalFromVision(double timestamp)
{
    if (mTurretMode != TurretControlModes::VISION_AIMED)
    {
        resetAimingParameters();
        return;
    }

    std::shared_ptr<Rotation2D> angle = Subsystems::LimelightManager::getInstance()->getTurretLimelight()->getAngleToTarget();
    
    mGoal.state.turret = mCurrentState.turret + angle->getDegrees();
    
    if (mGoal.state.turret < mTurret->getMinUnits())
        {
            mGoal.state.turret = mTurret->getMinUnits();
        } 
        if (mGoal.state.turret > mTurret->getMaxUnits())
        {
            mGoal.state.turret = mTurret->getMaxUnits();
        }
        mHasTarget = true;
    /*
    mLatestAimingParameters = mRobotState->getAimingParameters(true, -1, Constants::kMaxGoalTrackAge);
    if (mLatestAimingParameters.getRange() != 0.0)
    {
        mTrackId = mLatestAimingParameters.getTrackID();

        double kLookaheadTime = .7;
        std::shared_ptr<Pose2D> robot_to_predicted_robot = mRobotState->getLatestFieldToVehicle()->inverse()->transformBy(mRobotState->getPredictedFieldToVehicle(kLookaheadTime));

        std::shared_ptr<Pose2D> predicted_robot_to_goal = robot_to_predicted_robot->inverse()->transformBy(mLatestAimingParameters.getRobotToGoal());
        mCorrectedRangeToTarget = predicted_robot_to_goal->getTranslation()->norm();

        if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance)
        {
            return;
        }

        std::shared_ptr<Rotation2D> turret_error = mRobotState->getVehicleToTurret(timestamp)->getRotation()->inverse()->rotateBy(mLatestAimingParameters.getRobotToGoalRotation());

        mGoal.state.turret = mCurrentState.turret + turret_error->getDegrees();
        std::shared_ptr<Twist2D> velocity = mRobotState->getMeasuredVelocity();
        //angular velocity component from tangential robot motion about the goal
        double tangential_component = mLatestAimingParameters.getRobotToGoalRotation()->sin() * velocity->dx / mLatestAimingParameters.getRange();
        double angular_component = velocity->dtheta * Constants::kRadsToDegrees;
        //Add (opposite) of tangential velocity about goal + angular velocity in local frame.
        mTurretFeedforwardV = -(angular_component + tangential_component);

        if (mGoal.state.turret < mTurret->getMinUnits())
        {
            mGoal.state.turret = mTurret->getMinUnits();
        } 
        if (mGoal.state.turret > mTurret->getMaxUnits())
        {
            mGoal.state.turret = mTurret->getMaxUnits();
        }
        mHasTarget = true;

        if (util.epsilonEquals(turret_error->getDegrees(), 0.0, 3.0))
        {
            mOnTarget = true;
        } else
        {
            mOnTarget = false;
        }   
    
    } else
    {
        mHasTarget = false;
        mOnTarget = false;
    }
    */
    
}

void Superstructure::resetAimingParameters()
{
    mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardV = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = VisionTargeting::AimingParameters{};
}

double Superstructure::getCorrectedRangeToTarget()
{
    return mCorrectedRangeToTarget;
}

VisionTargeting::AimingParameters Superstructure::getLatestAimingParameters()
{
    return mLatestAimingParameters;
}

bool Superstructure::isOnTarget()
{
    return mOnTarget;
}

bool Superstructure::getCurrentlyAiming()
{
    return mTurretMode == TurretControlModes::VISION_AIMED;
}

int Superstructure::getTrackId()
{
    if(getCurrentlyAiming())
    {
        return mTrackId;
    }
    return -1;
}

void Superstructure::updateCurrentState()
{
    mCurrentState.turret = mTurret->getAngle();
    mCurrentState.hood = mHood->getAngle();
    mCurrentState.shooter = mShooter->getVelocity();
    mCurrentState.ballPathTop = mBallPathTop->getSetpoint();
    mCurrentState.centeringIntake = mCenteringIntake->getSetpoint();

    mCurrentState.numBalls = mBallPathTop->getBallCount();
    mCurrentState.extendIntake = mIntakeExtended;
    mCurrentState.extendWheelieBar = mWheelieBarExtended;
}

void Superstructure::setWantAutoAim(std::shared_ptr<Rotation2D> field_to_turret_hint, bool enforce_min_distance, double min_distance)
{
    mTurretMode = TurretControlModes::VISION_AIMED;
    mFieldRelativeGoal = field_to_turret_hint;
    mEnforceAutoAimMinDistance = enforce_min_distance;
    mAutoAimMinDistance = min_distance;
}

void Superstructure::setWantAutoAim(std::shared_ptr<Rotation2D> field_to_turret_hint)
{
    setWantAutoAim(field_to_turret_hint, false, 12.0);
}

void Superstructure::setWantRobotRelativeTurret()
{
    mTurretMode = TurretControlModes::ROBOT_RELATIVE;
}

void Superstructure::setWantFieldRelativeTurret(std::shared_ptr<Rotation2D> field_to_turret)
{
    mTurretMode = TurretControlModes::FIELD_RELATIVE;
    mFieldRelativeGoal = field_to_turret;
}

void Superstructure::setTurretOpenLoop(double throttle)
{
    mTurretMode = TurretControlModes::OPEN_LOOP;
    mTurretThrottle = throttle;   
}

void Superstructure::followSetpoint()
{
    frc::SmartDashboard::PutNumber("Superstructure Turret Goal: ", mGoal.state.turret);
    frc::SmartDashboard::PutNumber("Superstructure Hood Goal: ", mGoal.state.hood);
    frc::SmartDashboard::PutNumber("Superstructure Centering Intake Goal: ", mGoal.state.centeringIntake);
    frc::SmartDashboard::PutNumber("Superstructure BallPath Top Goal: ", mGoal.state.ballPathTop);
    frc::SmartDashboard::PutNumber("Superstructure Shooter Goal: ", mGoal.state.shooter);

    frc::SmartDashboard::PutBoolean("Superstructure Extend Intake Goal: ", mGoal.state.turret);
    frc::SmartDashboard::PutBoolean("Superstructure Extend Wheelie Bar Goal: ", mGoal.state.turret);

    frc::SmartDashboard::PutNumber("Superstructure numBalls Goal: ", mGoal.state.numBalls);
    
    frc::SmartDashboard::PutNumber("Superstructure Turret Mode: ", mTurretMode);
    double turret_offset = frc::SmartDashboard::GetNumber("Subsystems /" + mTurret->mConstants->kName + "/ Turret Offset degrees", 0.0);
    if (mTurretMode == TurretControlModes::JOGGING)
    {
        mTurret->setSetpointPositionPID(mGoal.state.turret, mTurretFeedforwardV);
    } //else if (util.epsilonEquals(mGoal.state.turret, mCurrentState.turret, mTurret->mConstants->kAllowableClosedLoopError / mTurret->mConstants->kTicksPerUnitDistance) )
    //{
    //    mTurret->setOpenLoop(0.0);
    //}
    else if (mTurretMode == TurretControlModes::VISION_AIMED )
    {
        mTurret->setSetpointPositionPID(mGoal.state.turret + turret_offset, mTurretFeedforwardV);
    } else if (mTurretMode == TurretControlModes::OPEN_LOOP)
    {
        mTurret->setOpenLoop(mTurretThrottle);
    } else
    { //robot or field relative (anything bigger than a few degrees)
        mTurret->setSetpointMotionMagic(mGoal.state.turret);
    }
    
    //if (util.epsilonEquals(mGoal.state.hood, mCurrentState.hood, mHood->mConstants->kAllowableClosedLoopError / mHood->mConstants->kTicksPerUnitDistance))
    //{
    //    mHood->setOpenLoop(0.0);
    //} else
    //{
        mHood->setSetpointPositionPID(mGoal.state.hood, mHoodFeedforwardV);
    //}

    mShooter->setSetpointVelocityPID(mGoal.state.shooter, mShooterFeedforwardV);
    frc::SmartDashboard::PutBoolean("BallPathManual", ballPathManual);
    if (!ballPathManual)
    {
        mBallPathTop->setOpenLoop(mGoal.state.ballPathTop);
    }
    
    mCenteringIntake->setOpenLoop(mGoal.state.centeringIntake);

    if (mGoal.state.extendIntake)
    {
        extendIntake();
    } else
    {
        stowIntake();
    }
    
}

void Superstructure::updateWantedAction()
{ //update wanted action if it is changed by transition
    StateMachines::SuperstructureStateMachine::WantedAction intakeAction = SystemStateToWantedAction(mStateMachine.getSystemIntakeState());
    StateMachines::SuperstructureStateMachine::WantedAction shooterAction = SystemStateToWantedAction(mStateMachine.getSystemShooterState());

    if (shooterAction != mWantedActionShooter)
    {
        mWantedActionShooter = shooterAction;
    }
    if (intakeAction != mWantedActionIntake)
    {
        mWantedActionIntake = intakeAction;
    }
}

StateMachines::SuperstructureStateMachine::WantedAction Superstructure::SystemStateToWantedAction(StateMachines::SuperstructureStateMachine::SystemState state)
{
    switch (state) {
        case StateMachines::SuperstructureStateMachine::IDLE:
            return StateMachines::SuperstructureStateMachine::WANTED_IDLE;
        case StateMachines::SuperstructureStateMachine::INTAKING_BALL:
            return StateMachines::SuperstructureStateMachine::WANTED_INTAKE_BALL;
        case StateMachines::SuperstructureStateMachine::PRE_EXHAUSTING_BALL:
            return StateMachines::SuperstructureStateMachine::WANTED_PRE_EXHAUST_BALL;
        case StateMachines::SuperstructureStateMachine::EXHAUSTING_BALL:
            return StateMachines::SuperstructureStateMachine::WANTED_EXHAUST_BALL;
        case StateMachines::SuperstructureStateMachine::HAVE_BALLS:
            return StateMachines::SuperstructureStateMachine::WANTED_HAVE_BALLS;
    }

    return StateMachines::SuperstructureStateMachine::WANTED_IDLE;
}

bool Superstructure::isAtDesiredState()
{
    return mGoal.isAtDesiredState(mCurrentState);
}

bool Superstructure::isShooting()
{
    return !(util.epsilonEquals( mGoal.state.shooter, 0.0));
}

void Superstructure::extendIntake()
{
    if (!mIntakeExtended)
    {
        mIntake.Set(true);
        mIntakeExtended = true;
    }
    
}

void Superstructure::stowIntake()
{
    if (mIntakeExtended)
    {
        mIntake.Set(false);
        mIntakeExtended = false;
    }
    
}

void Superstructure::setGoalNumBalls(double ballsToShoot)
{
    double initial_ball_count = (double) mBallPathTop->getBallCount() - ballsToShoot;
    double final_ball_count;
    if (initial_ball_count < 0)
    {
        final_ball_count = 0.0;
    } else
    {
        final_ball_count = initial_ball_count;
    }
    
    
    mStateMachine.setBallGoal(final_ball_count);
}

void Superstructure::setStateMachineLEDPriority(bool priority)
{
    mStateMachine.setLEDPriority(priority);
}

void Superstructure::setStateMachineLEDMaxPriority(bool maxPriority)
{
    mStateMachine.setLEDMaxPriority(maxPriority);
}