/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Kinematics.h"

Kinematics::Kinematics() {}

shared_ptr<Twist2D> Kinematics::forwardKinematics(double left_wheel_delta, double right_wheel_delta){
    double delta_rotation = (right_wheel_delta-left_wheel_delta)/ (constants->kDriveWheelTrackWidthInches*constants->kTrackScrubFactor);
    return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
}

shared_ptr<Twist2D> Kinematics::forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads){
    double dx= (left_wheel_delta+right_wheel_delta)/2.0;
    shared_ptr<Twist2D> tmp = make_shared<Twist2D>(dx, 0.0, delta_rotation_rads);
    return tmp;
}

shared_ptr<Twist2D> Kinematics::forwardKinematics(shared_ptr<Rotation2D> prev_heading, double left_wheel_delta, double right_wheel_delta, shared_ptr<Rotation2D> current_heading){
    double dx= (left_wheel_delta+right_wheel_delta)/2.0;
    shared_ptr<Twist2D> tmp = make_shared<Twist2D>(dx, 0.0, prev_heading->inverse()->rotateBy(current_heading)->getRadians());
    return tmp;
}

shared_ptr<Pose2D> Kinematics::integrateForwardKinematics(shared_ptr<Pose2D> current_pose, shared_ptr<Twist2D> forward_kinematics){
    return current_pose->transformBy(current_pose->exp(forward_kinematics));
}