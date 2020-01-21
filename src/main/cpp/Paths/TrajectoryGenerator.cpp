/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Paths/TrajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator() {
    
}

TrajectoryGenerator::TrajectorySet::TrajectorySet(){
    kMaxVelocity=Robot::constants.kDriveMaxVelocity;
    kMaxAccel=Robot::constants.kDriveMaxAcceleration;
    kMaxCentripetalAccel=Robot::constants.kDriveMaxCentripetalAcceleration;
    kMaxVoltage=Robot::constants.kDriveMaxVoltage;
    
    DriveForwardStraightTest=make_shared<MirroredTrajectory>(getDriveForwardStraightTest());
    DriveForwardSwerveTest=make_shared<MirroredTrajectory>(getDriveForwardSwerveTest());
    DriveForwardSwerveAngledTest=make_shared<MirroredTrajectory>(getDriveForwardSwerveAngledTest());

    DriveReverseStraightTest=make_shared<MirroredTrajectory>(getDriveReverseStraightTest());
    DriveReverseSwerveTest=make_shared<MirroredTrajectory>(getDriveReverseSwerveTest());
    DriveReverseSwerveAngledTest=make_shared<MirroredTrajectory>(getDriveReverseSwerveAngledTest());


}

TrajectoryGenerator::TrajectorySet::MirroredTrajectory::MirroredTrajectory(vector<shared_ptr<TimedState>> right){
    mRight=right;
    mLeft=trajectoryUtil->mirrorTimed(right);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::generateTrajectories(
    bool reversed,
    vector<shared_ptr<Pose2D>> waypoints,
    double start_vel, //inches/s
    double end_vel, //inches/s
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage){
        shared_ptr<DriveMotionPlanner> mMotionPlanner=make_shared<DriveMotionPlanner>();
        return mMotionPlanner->generateTrajectory(reversed, waypoints, start_vel, end_vel, max_vel, max_accel, max_voltage);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::generateTrajectories(
    bool reversed,
    vector<shared_ptr<Pose2D>> waypoints,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage){
        shared_ptr<DriveMotionPlanner> mMotionPlanner=make_shared<DriveMotionPlanner>();
        return mMotionPlanner->generateTrajectory(reversed, waypoints, max_vel, max_accel, max_voltage);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::getDriveForwardStraightTest(){
    vector<shared_ptr<Pose2D>> waypoints;
    waypoints.push_back(origin);
    waypoints.push_back(forwardStraightEnd);

    return generateTrajectories(false,waypoints, kMaxVelocity, kMaxAccel, kMaxVoltage);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::getDriveForwardSwerveTest(){
    vector<shared_ptr<Pose2D>> waypoints;
    waypoints.push_back(origin);
    waypoints.push_back(forwardSwerveEnd);

    return generateTrajectories(false,waypoints, kMaxVelocity, kMaxAccel, kMaxVoltage);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::getDriveForwardSwerveAngledTest(){
    vector<shared_ptr<Pose2D>> waypoints;
    waypoints.push_back(origin);
    waypoints.push_back(forwardSwerveAngledEnd);

    return generateTrajectories(false,waypoints, kMaxVelocity, kMaxAccel, kMaxVoltage);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::getDriveReverseStraightTest(){
    vector<shared_ptr<Pose2D>> waypoints;
    waypoints.push_back(forwardStraightEnd);
    waypoints.push_back(origin);

    return generateTrajectories(true,waypoints, kMaxVelocity, kMaxAccel, kMaxVoltage);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::getDriveReverseSwerveTest(){
    vector<shared_ptr<Pose2D>> waypoints;
    waypoints.push_back(forwardSwerveEnd);
    waypoints.push_back(origin);

    return generateTrajectories(true,waypoints, kMaxVelocity, kMaxAccel, kMaxVoltage);
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::getDriveReverseSwerveAngledTest(){
    vector<shared_ptr<Pose2D>> waypoints;
    waypoints.push_back(forwardSwerveAngledEnd);
    waypoints.push_back(origin);

    return generateTrajectories(true,waypoints, kMaxVelocity, kMaxAccel, kMaxVoltage);
}

void TrajectoryGenerator::generateTrajectories(){
    Trajectories=make_shared<TrajectorySet>();
}

vector<shared_ptr<TimedState>> TrajectoryGenerator::TrajectorySet::MirroredTrajectory::get(bool left){
    return (left? mLeft:mRight);
}