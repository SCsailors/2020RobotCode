/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Superstructure.h"
#include "Constants.h"
using namespace Subsystems;
std::shared_ptr<Subsystems::Superstructure> Superstructure::mInstance;

Superstructure::Superstructure() 
{
    mBallPathBottom = Subsystems::BallPathBottom::getInstance();
    mBallPathTop = Subsystems::BallPathTop::getInstance();
    mCenteringIntake = Subsystems::CenteringIntake::getInstance();
    mHood = Subsystems::Hood::getInstance();
    mShooter = Subsystems::Shooter::getInstance();
    mTurret = Subsystems::Turret::getInstance();
    mRobotState = FRC_7054::RobotState::getInstance();
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
    updateCurrentState();
    maybeUpdateGoalFromVision(timestamp);
    maybeUpdateGoalFromFieldRelativeGoal(timestamp);
    followSetpoint();
}

void Superstructure::OnStop(double timestamp){}

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
    mGoal.state.turret = mCurrentState.turret + delta;
    mTurretFeedforwardV = 0.0;
}

void Superstructure::setGoal(SuperstructureGoal goal)
{
    if (mTurretMode == TurretControlModes::VISION_AIMED && mHasTarget)
    {
        //keep existing turret setpoint.
    } else
    {
        mGoal.state.turret = goal.state.turret;
    }

    mGoal.state.hood = goal.state.hood;
    mGoal.state.shooter = goal.state.shooter;
    mGoal.state.centeringIntake = goal.state.centeringIntake;
    mGoal.state.ballPathBottom = goal.state.ballPathBottom;
    mGoal.state.ballPathTop = goal.state.ballPathTop;
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

    if (mGoal.state.turret < Constants::kTurretConstants->kMinUnitsLimit)
    {
        mGoal.state.turret += 360.0;
    } 
    if (mGoal.state.turret > Constants::kTurretConstants->kMaxUnitsLimit)
    {
        mGoal.state.turret -=360.0;
    }

}

void Superstructure::maybeUpdateGoalFromVision(double timestamp)
{
    if (mTurretMode != TurretControlModes::VISION_AIMED)
    {
        resetAimingParameters();
        return;
    }

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

        if (mGoal.state.turret < Constants::kTurretConstants->kMinUnitsLimit)
        {
            mGoal.state.turret += 360.0;
        } 
        if (mGoal.state.turret > Constants::kTurretConstants->kMaxUnitsLimit)
        {
            mGoal.state.turret -=360.0;
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
    mCurrentState.ballPathBottom = mBallPathBottom->getVelocity();
    mCurrentState.ballPathTop = mBallPathTop->getVelocity();
    mCurrentState.centeringIntake = mCenteringIntake->getVelocity();
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
    if (mTurretMode == TurretControlModes::VISION_AIMED || mTurretMode == TurretControlModes::JOGGING)
    {
        mTurret->setSetpointPositionPID(mGoal.state.turret, mTurretFeedforwardV);
    } else if (mTurretMode == TurretControlModes::OPEN_LOOP)
    {
        mTurret->setOpenLoop(mTurretThrottle);
    } else
    {
        mTurret->setSetpointMotionMagic(mGoal.state.turret);
    }

    //maybe add something here about hood and trench
    mHood->setSetpointPositionPID(mGoal.state.hood, mHoodFeedforwardV);
    mShooter->setSetpointVelocityPID(mGoal.state.shooter, mShooterFeedforwardV);
    mBallPathBottom->setSetpointVelocityPID(mGoal.state.ballPathBottom, mBallPathBottomFeedforwardV);
    mBallPathTop->setSetpointVelocityPID(mGoal.state.ballPathTop, mBallPathTopFeedforwardV);
    mCenteringIntake->setSetpointVelocityPID(mGoal.state.centeringIntake, mCenteringIntakeFeedforwardV);
}

bool Superstructure::isAtDesiredState()
{
    return mGoal.isAtDesiredState(mCurrentState);
}

bool Superstructure::isShooting()
{
    return util.epsilonEquals( mGoal.state.shooter, 0.0);
}

void Superstructure::extendWheelieBar()
{
    mWheelieBar.Set(true);
}

void Superstructure::stowWheelieBar()
{
    mWheelieBar.Set(false);
}

void Superstructure::extendIntake()
{
    mIntake.Set(true);
}

void Superstructure::stowIntake()
{
    mIntake.Set(false);
}