/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/Timing/TimingConstraint.h"
#include "Robot.h"




TimingConstraint::MinMaxAcceleration::MinMaxAcceleration(){
    min_acceleration_=-1E100;
    max_acceleration_=1E100;
    //kNoLimits=make_shared<MinMaxAcceleration>();
}

TimingConstraint::MinMaxAcceleration::MinMaxAcceleration(double min_accel, double max_accel){
    max_acceleration_=max_accel;
    min_acceleration_=min_accel;
    //kNoLimits=make_shared<MinMaxAcceleration>();
}

double TimingConstraint::MinMaxAcceleration::min_acceleration(){
    return min_acceleration_;
}

double TimingConstraint::MinMaxAcceleration::max_acceleration(){
    return max_acceleration_;
}

bool TimingConstraint::MinMaxAcceleration::valid(){
    return min_acceleration_<=max_acceleration_;
}

TimingConstraint::CentripetalAccelerationConstraint::CentripetalAccelerationConstraint(double max_centripetal_acceleration){
    max_centripetal_acceleration_=max_centripetal_acceleration;
}

double TimingConstraint::CentripetalAccelerationConstraint::getMaxVelocity(shared_ptr<Pose2DWithCurvature> state){
    if (Robot::util.epsilonEquals(state->getCurvature(), 0.0)){
        return 1E100; // TODO fix this in POSE2DWC
    }
    
    return std::sqrt(std::fabs(max_centripetal_acceleration_/state->getCurvature()));
}

shared_ptr<TimingConstraint::MinMaxAcceleration> TimingConstraint::CentripetalAccelerationConstraint::getMinMaxAcceleration(shared_ptr<Pose2DWithCurvature> state, double velocity){
    //shared_ptr<MinMaxAcceleration> Accel= make_shared<MinMaxAcceleration>();
    //return Accel->kNoLimits;
    return make_shared<TimingConstraint::MinMaxAcceleration>();

}

TimingConstraint::DifferentialDriveDynamicConstraints::DifferentialDriveDynamicConstraints(shared_ptr<DifferentialDrive> drive, double abs_voltage_limit){
    drive_=drive;
    abs_voltage_limit_=abs_voltage_limit;
}

double TimingConstraint::DifferentialDriveDynamicConstraints::getMaxVelocity(shared_ptr<Pose2DWithCurvature> state){
    return Robot::units.meters_to_inches(drive_->getMaxAbsVelocity(
        Robot::units.meters_to_inches(state->getCurvature()), //correct because curvature is inverse inches
        abs_voltage_limit_));
}

shared_ptr<TimingConstraint::MinMaxAcceleration> TimingConstraint::DifferentialDriveDynamicConstraints::getMinMaxAcceleration(shared_ptr<Pose2DWithCurvature> state, double velocity){
    shared_ptr<DifferentialDrive::ChassisState> ChassisState_= make_shared<DifferentialDrive::ChassisState>(Robot::units.inches_to_meters(velocity), state->getCurvature()*velocity);
    shared_ptr<DifferentialDrive::MinMax> min_max=make_shared<DifferentialDrive::MinMax>(); 
    min_max=min_max->getMinMaxAcceleration(ChassisState_, Robot::units.meters_to_inches(state->getCurvature()), abs_voltage_limit_);
    shared_ptr<MinMaxAcceleration> Min_Max = make_shared<MinMaxAcceleration>(Robot::units.meters_to_inches(min_max->min), Robot::units.meters_to_inches(min_max->max));
    return Min_Max;
}

TimingConstraint::TimingConstraint(double max_centripetal_acceleration, shared_ptr<DifferentialDrive> drive, double abs_voltage_limit) {
     CAC= make_shared<TimingConstraint::CentripetalAccelerationConstraint>(max_centripetal_acceleration);
     DDDC= make_shared<TimingConstraint::DifferentialDriveDynamicConstraints>(drive, abs_voltage_limit);

}

double TimingConstraint::getMaxVelocity(shared_ptr<Pose2DWithCurvature> state){
    return fmin(CAC->getMaxVelocity(state), DDDC->getMaxVelocity(state));
}

shared_ptr<TimingConstraint::MinMaxAcceleration> TimingConstraint::getMinMaxAcceleration(shared_ptr<Pose2DWithCurvature> state, double velocity){ 
    return make_shared<TimingConstraint::MinMaxAcceleration>();
}