/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/RobotStateEstimator.h"
#include "Robot.h"
#include "lib/Geometry/Pose2D.h"
using namespace Subsystems;
namespace Subsystems{
RobotStateEstimator::RobotStateEstimator() {
}


void RobotStateEstimator::OnStart(double timestamp){
    
    left_encoder_prev_distance_=Robot::drive->getLeftEncoderDistance(); 
    right_encoder_prev_distance_=Robot::drive->getRightEncoderDistance(); 
    frc::SmartDashboard::PutNumber("Left Encoder prev distance", left_encoder_prev_distance_);
    frc::SmartDashboard::PutNumber("Right Encoder prev distance", right_encoder_prev_distance_);
}

void RobotStateEstimator::OnLoop(double timestamp){
    double left_distance= Robot::drive->getLeftEncoderDistance();
    double right_distance= Robot::drive->getRightEncoderDistance();
    double delta_left= left_distance-left_encoder_prev_distance_;
    double delta_right= right_distance-right_encoder_prev_distance_;
    shared_ptr<Rotation2D> gyro_angle = Robot::drive->getHeading();
    shared_ptr<Twist2D> odometry_velocity = Robot::robotState.generateOdometryFromSensors(delta_left, delta_right, gyro_angle); 
    shared_ptr<Twist2D> predicted_velocity = Robot::kinematics.forwardKinematics(Robot::drive->getLeftLinearVelocity(), Robot::drive->getRightLinearvelocity());
    Robot::robotState.addObservation(timestamp, odometry_velocity, predicted_velocity);
    left_encoder_prev_distance_= left_distance;
    right_encoder_prev_distance_= right_distance;
}

void RobotStateEstimator::OnStop(double timestamp){
    //no operations
}

bool RobotStateEstimator::checkSystem(){
    return true;
}

void RobotStateEstimator::outputTelemetry(){
    //no operations
}

void RobotStateEstimator::readPeriodicInputs(){
    //no ops
}

void RobotStateEstimator::writePeriodicOutputs(){
    //no ops
}

void RobotStateEstimator::zeroSensors(){
    //Reset RobotState
    Robot::robotState.reset(0.0, make_shared<Pose2D>());
}

void RobotStateEstimator::writeToLog(){
    //no ops
}
}