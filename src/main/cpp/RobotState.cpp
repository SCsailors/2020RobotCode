/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotState.h"
#include "Robot.h"
#include <cmath>
#include <Constants.h>

std::shared_ptr<FRC_7054::RobotState> FRC_7054::RobotState::mInstance;
 
FRC_7054::RobotState::RobotState() {
    shared_ptr<Pose2D> tmp=make_shared<Pose2D>();
     reset(0.0, tmp, tmp);
}

std::shared_ptr<FRC_7054::RobotState> FRC_7054::RobotState::getInstance()
{
    if (!mInstance)
    {
        mInstance = std::make_shared<FRC_7054::RobotState>();
    }

    return mInstance;
}

void FRC_7054::RobotState::reset(double start_time, shared_ptr<Pose2D> initial_field_to_vehicle, shared_ptr<Pose2D> initial_vehicle_to_turret)
{
    reset(start_time, initial_field_to_vehicle);
    vehicle_to_turret->put(start_time, initial_vehicle_to_turret);
}

void FRC_7054::RobotState::reset(double start_time, shared_ptr<Pose2D> initial_field_to_vehicle){
    field_to_vehicle->put(start_time, initial_field_to_vehicle);
}

void FRC_7054::RobotState::reset()
{
    shared_ptr<Pose2D> tmp=make_shared<Pose2D>();
    reset(frc::Timer::GetFPGATimestamp(), tmp, tmp);
}

shared_ptr<Pose2D> FRC_7054::RobotState::getFieldToVehicle(double timestamp){
    return field_to_vehicle->getInterpolated(timestamp);
}

shared_ptr<Pose2D> FRC_7054::RobotState::getVehicleToTurret(double timestamp)
{
    return vehicle_to_turret->getInterpolated(timestamp);
}

shared_ptr<Pose2D> FRC_7054::RobotState::getFieldToTurret(double timestamp)
{
    return getFieldToVehicle(timestamp)->transformBy(getVehicleToTurret(timestamp));
}

shared_ptr<Pose2D> FRC_7054::RobotState::getLatestFieldToVehicle(){
    return field_to_vehicle->pastStates.at(field_to_vehicle->pastStates.size()-1)->state_;
}

shared_ptr<Pose2D> FRC_7054::RobotState::getLatestVehicleToTurret()
{
    return vehicle_to_turret->pastStates.at(vehicle_to_turret->pastStates.size()-1)->state_;
}

shared_ptr<Pose2D> FRC_7054::RobotState::getPredictedFieldToVehicle(double lookahead_time){
    return getLatestFieldToVehicle()->transformBy(pose->exp(vehicle_velocity_predicted_->scaled(lookahead_time)));
}

void FRC_7054::RobotState::addFieldToVehicleObservation(double timestamp, shared_ptr<Pose2D> observation){
    field_to_vehicle->put(timestamp, observation);
}

void FRC_7054::RobotState::addVehicleToTurretObservation(double timestamp, shared_ptr<Pose2D> observation)
{
    vehicle_to_turret->put(timestamp, observation);
}

void FRC_7054::RobotState::addObservation(double timestamp, shared_ptr<Twist2D> measured_velocity, shared_ptr<Twist2D> predicted_velocity){
    addFieldToVehicleObservation(timestamp, kinematics.integrateForwardKinematics(getLatestFieldToVehicle(), measured_velocity));
    vehicle_velocity_measured_=measured_velocity;
    vehicle_velocity_predicted_=predicted_velocity;

    if (fabs(vehicle_velocity_measured_->dtheta) < 2.0 *Constants::kPI)
    {
        //reject really high angular velocities from filter
        vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
    } else 
    {
        shared_ptr<Twist2D> twist = make_shared<Twist2D>(vehicle_velocity_measured_->dx, vehicle_velocity_measured_->dy, 0.0);
        vehicle_velocity_measured_filtered_.add(twist);
    }

    vehicle_acceleration_measured_ = vehicle_velocity_measured_->derive(prev_vehicle_acceleration_, timestamp - prev_timestamp);
    
    //check if limits are needed    
    vehicle_acceleration_measured_filtered_.add(vehicle_acceleration_measured_);

    prev_vehicle_acceleration_ = vehicle_acceleration_measured_;
    prev_timestamp = timestamp;
}

shared_ptr<Twist2D> FRC_7054::RobotState::generateOdometryFromSensors(double left_encoder_delta_distance, double right_encoder_delta_distance, shared_ptr<Rotation2D> current_gyro_angle){
    shared_ptr<Pose2D> last_measurement= getLatestFieldToVehicle();
    shared_ptr<Twist2D> delta= kinematics.forwardKinematics(last_measurement->getRotation(), left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
    distance_driven_+=delta->dx;
    return delta;
}

double FRC_7054::RobotState::getDistanceDriven(){
    return distance_driven_;
}

void FRC_7054::RobotState::resetDistanceDriven(){
    distance_driven_=0.0;
}

shared_ptr<Twist2D> FRC_7054::RobotState::getPredictedVelocity(){
    return vehicle_velocity_predicted_;
}

shared_ptr<Twist2D> FRC_7054::RobotState::getMeasuredVelocity(){
    return vehicle_velocity_measured_;
}

shared_ptr<Twist2D> FRC_7054::RobotState::getSmoothedVelocity()
{
    return vehicle_velocity_measured_filtered_.getAverage();
}

shared_ptr<Twist2D> FRC_7054::RobotState::getMeasuredAcceleration()
{
    return vehicle_acceleration_measured_;
}

shared_ptr<Twist2D> FRC_7054::RobotState::getSmoothedAcceleration()
{
    return vehicle_acceleration_measured_filtered_.getAverage();
}

void FRC_7054::RobotState::resetVision()
{
    vision_target_intake.reset();
    vision_target_turret.reset();
}

std::shared_ptr<Pose2D> FRC_7054::RobotState::getCameraToVisionTargetPose(VisionTargeting::TargetInfo target, bool turret, std::shared_ptr<Subsystems::Limelight> source)
{
    //change coordinate frames: x (left, right), y (up, down), z (forwards, backwards) to x (forward, backwards), y (left, right), z (up, down)
    double x = target.getZ(); // depth
    double y = target.getX(); // left right
    double z = target.getY(); // height

    //if (util.epsilonEquals())
    std::shared_ptr<Pose2D> pose = std::make_shared<Pose2D>(x, y, 0.0);
    return pose;
}

void FRC_7054::RobotState::updateGoalTracker(double timestamp, std::vector<std::shared_ptr<Pose2D>> goalPose, VisionTargeting::GoalTracker goalTracker, std::shared_ptr<Subsystems::Limelight> source)
{
    std::vector<std::shared_ptr<Pose2D>> goalPoses;
    //check if z axis needs to be rotated by camera angle, should be rotated above
    if (goalPose.empty())
    {
        std::cout<<"failed to update Goal Tracker: " << source->getName() <<std::endl;
        return;
    }
    for (auto pose : goalPose)
    {
        std::shared_ptr<Pose2D> fieldToVisionTarget = getFieldToTurret(timestamp)->transformBy(source->getTurretToLens()->transformBy(pose));
        goalPoses.push_back(fieldToVisionTarget);
    }
    goalTracker.update(timestamp, goalPoses);
    
}

void FRC_7054::RobotState::addVisionUpdate(double timestamp, std::vector<VisionTargeting::TargetInfo> observations, std::vector<std::shared_ptr<Subsystems::Limelight>> limelights)
{
    if (observations.empty())
    {
        frc::SmartDashboard::PutBoolean("CheckPoint/ VisionUpdate/ empty", true);
        //std::cout << "Removing Old Targets: observations empty" << std::endl;
        //empty update Goal trackers to remove old targets
        vision_target_intake.update(timestamp, emptyVector);
        vision_target_turret.update(timestamp, emptyVector);
        return;
    }

    mCameraToVisionTargetPosesIntake.clear();
    mCameraToVisionTargetPosesTurret.clear();
    
    for (auto target : observations)
    {
        mCameraToVisionTargetPosesTurret.push_back( getCameraToVisionTargetPose(target, true, limelights.at(0) ) );
        mCameraToVisionTargetPosesIntake.push_back( getCameraToVisionTargetPose(target, false, limelights.at(1) ) );
    }
    frc::SmartDashboard::PutBoolean("CheckPoint/ VisionUpdate/ added xyz poses", true);
    
    
    //update Goal trackers with actual data
    updateGoalTracker(timestamp, mCameraToVisionTargetPosesTurret, vision_target_turret, limelights.at(0));
    updateGoalTracker(timestamp, mCameraToVisionTargetPosesIntake, vision_target_intake, limelights.at(1));    
    
}

std::shared_ptr<Pose2D> FRC_7054::RobotState::getFieldToVisionTarget(bool turret)
{
    VisionTargeting::GoalTracker tracker = turret ? vision_target_turret : vision_target_intake;

    if (!tracker.hasTracks())
    {
        return NULL;
    }

    std::shared_ptr<Pose2D> fieldToTarget = tracker.getTracks().at(0).field_to_target;

    double normalPositive = fmod(fieldToTarget->getRotation()->getDegrees() + 360.0, 360.0); 
    double normalClamped = kPossibleNormals.at(0);
    for (double possible : kPossibleNormals)
    {
        if (fabs(normalPositive - possible) < fabs(normalPositive - normalClamped))
        {
            normalClamped = possible;
        }
    }
    std::shared_ptr<Pose2D> pose = std::make_shared<Pose2D>(fieldToTarget->getTranslation(), fieldToTarget->getRotation()->fromDegrees(normalClamped));
    return pose;
}

std::shared_ptr<Pose2D> FRC_7054::RobotState::getVehicleToVisionTarget(double timestamp, bool turret)
{
    std::shared_ptr<Pose2D> fieldToVisionTarget = getFieldToVisionTarget(turret);

    if (fieldToVisionTarget == NULL)
    {
        return NULL;
    }

    return getFieldToVehicle(timestamp)->inverse()->transformBy(fieldToVisionTarget);
}


VisionTargeting::AimingParameters FRC_7054::RobotState::getAimingParameters(bool turret, int prev_track_id, double max_track_age)
{
    VisionTargeting::GoalTracker tracker = turret ? vision_target_turret : vision_target_intake;

    std::vector<VisionTargeting::TrackReport> reports = tracker.getTracks();

    if (reports.empty())
    {
        return VisionTargeting::AimingParameters{};
    }

    double timestamp = frc::Timer::GetFPGATimestamp();

    VisionTargeting::TrackReportComparator comparator{Constants::kTrackStabilityWeight, Constants::kTrackAgeWeight, Constants::kTrackSwitchingWeight, prev_track_id, timestamp};

    VisionTargeting::TrackReport report{};

    for (auto track : reports)
    {
        if (track.latest_timestamp > timestamp - max_track_age)
        {
            if (comparator.compare(report, track) > 0)
            {
                report = track;
            }
        }
    }

    std::shared_ptr<Pose2D> vehicleToGoal = getFieldToVehicle(timestamp)->inverse()->transformBy(report.field_to_target);
    VisionTargeting::AimingParameters params{
        vehicleToGoal,
        report.field_to_target,
        report.field_to_target->getRotation(),
        report.latest_timestamp, report.stability, 
        report.id
    };
    return params;
}

void FRC_7054::RobotState::outputToSmartDashboard(){
    shared_ptr<Pose2D> odometry= getLatestFieldToVehicle();
    frc::SmartDashboard::PutNumber("Robot Pose X", odometry->getTranslation()->x());
    frc::SmartDashboard::PutNumber("Robot Pose Y", odometry->getTranslation()->y());
    frc::SmartDashboard::PutNumber("Robot Pose Theta", odometry->getRotation()->getDegrees());
    frc::SmartDashboard::PutNumber("Robot Velocity (X)", vehicle_velocity_measured_->dx);
    frc::SmartDashboard::PutNumber("Robot Velocity (Theta)", vehicle_velocity_measured_->dtheta);

    frc::SmartDashboard::PutNumber("Robot filtered Velocity (X)", vehicle_velocity_measured_filtered_.getAverage()->dx);
    frc::SmartDashboard::PutNumber("Robot filtered Velocity (Theta)", vehicle_velocity_measured_filtered_.getAverage()->dtheta);

    frc::SmartDashboard::PutNumber("Robot Acceleration (X)", vehicle_acceleration_measured_->dx);
    frc::SmartDashboard::PutNumber("Robot Acceleration (Theta)", vehicle_acceleration_measured_->dtheta);

    frc::SmartDashboard::PutNumber("Robot filtered Acceleration (X)", vehicle_acceleration_measured_filtered_.getAverage()->dx);
    frc::SmartDashboard::PutNumber("Robot filtered Acceleration (Theta)", vehicle_acceleration_measured_filtered_.getAverage()->dtheta);
}

string FRC_7054::RobotState::toPlannerCSV(){
    return toString(getLatestFieldToVehicle()->getTranslation()->x())+","+toString(getLatestFieldToVehicle()->getTranslation()->y())+","+toString(getLatestFieldToVehicle()->getRotation()->getDegrees())+",";
}

string FRC_7054::RobotState::toString(double value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}