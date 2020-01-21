/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotState.h"
 
RobotState::RobotState() {
    shared_ptr<Pose2D> tmp=make_shared<Pose2D>();
     reset(0.0, tmp);
}

void RobotState::reset(double start_time, shared_ptr<Pose2D> initial_field_to_vehicle){
    field_to_vehicle=make_shared<InterpolatingTreeMap>(kObservationBufferSize);
    field_to_vehicle->put(start_time, initial_field_to_vehicle);
    vehicle_velocity_measured_=make_shared<Twist2D>();
    vehicle_velocity_predicted_=make_shared<Twist2D>();
    distance_driven_=0.0;
}

void RobotState::resetDistanceDriven(){
    distance_driven_=0.0;
}

shared_ptr<Pose2D> RobotState::getFieldToVehicle(double timestamp){
    return field_to_vehicle->getInterpolated(timestamp);
}

shared_ptr<Pose2D> RobotState::getLatestFieldToVehicle(){
    return field_to_vehicle->pastStates.at(field_to_vehicle->pastStates.size()-1)->state_;
}

shared_ptr<Pose2D> RobotState::getPredictedFieldToVehicle(double lookahead_time){
    return getLatestFieldToVehicle()->transformBy(pose->exp(vehicle_velocity_predicted_->scaled(lookahead_time)));
}

void RobotState::addFieldToVehicleObservation(double timestamp, shared_ptr<Pose2D> observation){
    field_to_vehicle->put(timestamp, observation);
}

void RobotState::addObservation(double timestamp, shared_ptr<Twist2D> measured_velocity, shared_ptr<Twist2D> predicted_velocity){
    addFieldToVehicleObservation(timestamp, Robot::kinematics.integrateForwardKinematics(getLatestFieldToVehicle(), measured_velocity));
    vehicle_velocity_measured_=measured_velocity;
    vehicle_velocity_predicted_=predicted_velocity;
}

shared_ptr<Twist2D> RobotState::generateOdometryFromSensors(double left_encoder_delta_distance, double right_encoder_delta_distance, shared_ptr<Rotation2D> current_gyro_angle){
    shared_ptr<Pose2D> last_measurement= getLatestFieldToVehicle();
    shared_ptr<Twist2D> delta= Robot::kinematics.forwardKinematics(last_measurement->getRotation(), left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
    distance_driven_+=delta->dx;
    return delta;
}

double RobotState::getDistanceDriven(){
    return distance_driven_;
}

shared_ptr<Twist2D> RobotState::getMeasuredVelocity(){
    return vehicle_velocity_measured_;
}

shared_ptr<Twist2D> RobotState::getPredictedVelocity(){
    return vehicle_velocity_predicted_;
}

void RobotState::outputToSmartDashboard(){
    shared_ptr<Pose2D> odometry= getLatestFieldToVehicle();
    frc::SmartDashboard::PutNumber("Robot Pose X", odometry->getTranslation()->x());
    frc::SmartDashboard::PutNumber("Robot Pose Y", odometry->getTranslation()->y());
    frc::SmartDashboard::PutNumber("Robot Pose Theta", odometry->getRotation()->getDegrees());
    frc::SmartDashboard::PutNumber("Robot Linear Velocity", vehicle_velocity_measured_->dx);
}

string RobotState::toPlannerCSV(){
    return toString(getLatestFieldToVehicle()->getTranslation()->x())+","+toString(getLatestFieldToVehicle()->getTranslation()->y())+","+toString(getLatestFieldToVehicle()->getRotation()->getDegrees())+",";
}

string RobotState::toString(double value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}