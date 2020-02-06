/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Planners/DriveMotionPlanner.h"
#include "Robot.h"
#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "Constants.h"

DriveMotionPlanner::DriveMotionPlanner() {
    shared_ptr<DCMotorTransmission> transmission= make_shared<DCMotorTransmission>(
        1.0/Constants::kDriveKv,
        units->inches_to_meters(Constants::kDriveWheelRadiusInches)*units->inches_to_meters(Constants::kDriveWheelRadiusInches)*Constants::kRobotLinearInertia/(2.0*Constants::kDriveKa),
        Constants::kDriveVIntercept);

    mModel=make_shared<DifferentialDrive>(
        Constants::kRobotLinearInertia,
        Constants::kRobotAngularInertia,
        Constants::kRobotAngularDrag,
        units->inches_to_meters(Constants::kDriveWheelDiameterInches/2.0),
        units->inches_to_meters(Constants::kDriveWheelTrackWidthInches/2.0*Constants::kTrackScrubFactor),
        transmission, transmission);
    constraints= make_shared<TimingConstraint>(130.0, mModel, 9.0);
    //mCSVWriter=make_shared<CSVWriter>("motion_planning_logs", "Setpoint T,Setpoint X,Setpoint Y,Setpoint Theta,Setpoint Velocity,Setpoint Acceleration, Setpoint Curvature, Setpoint dCurvature_ds, Error X, Error Y, Error Theta, Linear Velocity, Angular Velocity, Adjusted Linear Velocity, Adjusted Angular Velocity, Left Feedforward, Right Feedforward, Adjusted Left Feedforward, Adjusted Right Feedforward, Actual Left Output, Actual Right Output, Pose X, Pose Y, Pose Theta, Left Demand, Right Demand, Left Velocity Actual, Right Velocity Actual");
    frc::SmartDashboard::PutNumber("MotionPlanner kBeta", 1.5);
    frc::SmartDashboard::PutNumber("MotionPlanner kZeta", .4);

}


void DriveMotionPlanner::setTrajectory(shared_ptr<TrajectoryIterator> trajectory){
    mCurrentTrajectory=trajectory;
    mSetpoint=trajectory->getState();
    for (shared_ptr<TimedState> state :trajectory->Trajectory()){
        if (state->velocity()>util->kEpsilon){
            mIsReversed=false;
            break;
        }else if (state->velocity()<-util->kEpsilon){
            mIsReversed=true;
            break;
        }
    }
}

void DriveMotionPlanner::reset(){
    mError=make_shared<Pose2D>();
    mOutput=make_shared<Output>();
    mLastTime=1E100;
}

vector<shared_ptr<TimedState>> DriveMotionPlanner::generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    double max_vel,
    double max_accel, 
    double max_voltage
){
    return generateTrajectory(reverse, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
}

vector<shared_ptr<TimedState>> DriveMotionPlanner::generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    double start_vel,
    double end_vel,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage
  ){
      return generateTrajectory(reverse, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
}

vector<shared_ptr<TimedState>> DriveMotionPlanner::generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    shared_ptr<TimingConstraint> constraints,
    double max_vel, //inches/s
    double max_accel, //inches/s^2
    double max_voltage
  ){
      return generateTrajectory(reverse, waypoints, constraints, max_vel, max_accel, max_voltage);
  }

vector<shared_ptr<TimedState>> DriveMotionPlanner::generateTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2D>> waypoints,
    shared_ptr<TimingConstraint> constraints,
    double start_vel,
    double end_vel,
    double max_vel,
    double max_accel, 
    double max_voltage
){
    double startTime=frc::Timer::GetFPGATimestamp();
    vector<shared_ptr<Pose2D>> waypoints_maybe_flipped;
    shared_ptr<Translation2D> trans= make_shared<Translation2D>();
    shared_ptr<Rotation2D> rot= make_shared<Rotation2D>(-1.0, 0.0, false);
    shared_ptr<Pose2D> flip = make_shared<Pose2D>(trans, rot);
    
    if(reverse){
        for(shared_ptr<Pose2D> waypoint: waypoints){
            waypoints_maybe_flipped.push_back(waypoint->transformBy(flip));

        }
    }else{
        waypoints_maybe_flipped=waypoints;
    }
    
    //create a trajectory from splines
    vector<shared_ptr<Pose2DWithCurvature>> trajectory= trajUtil->trajectoryFromSplineWaypoints(waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);
    
    if(reverse){
        for(shared_ptr<Pose2DWithCurvature> point: trajectory){
            shared_ptr<Pose2DWithCurvature> pos2=make_shared<Pose2DWithCurvature>(point->getPose()->transformBy(flip), -point->getCurvature(), point->getDCurvatureDs());
            flipped.push_back(pos2);
        }
        trajectory=flipped;
    }
    
    //generate timed trajectory
    vector<shared_ptr<TimedState>> timed_trajectory= timingUtil->timeParameterizeTrajectory(reverse, trajectory, constraints, start_vel, end_vel, max_vel, max_accel);
    frc::SmartDashboard::PutNumber("Trajectory Generation Time: ", frc::Timer::GetFPGATimestamp()-startTime);
    cout<<"Trajectory Generation Time: "<<frc::Timer::GetFPGATimestamp()-startTime<<endl;
    return timed_trajectory;
}

template<typename T>
string DriveMotionPlanner::toString(T value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}

string DriveMotionPlanner::getFields(){
    return "left_velocity, right_velocity, left_feedforward_voltage, right_feedforward_voltage";
}

string DriveMotionPlanner::toCSV(){
    return toString(mOutput->left_velocity) + ","+ toString(mOutput->right_velocity) + ","+ toString(mOutput->left_feedforward_voltage) + ","+toString(mOutput->right_feedforward_voltage);
} //TODO add mSetpoint data: need to add in Trajectory SamplePoint toCSV, toString, getFields()

DriveMotionPlanner::Output::Output(){}

DriveMotionPlanner::Output::Output(double leftVel, double rightVel, double leftAccel, double rightAccel, double leftVol, double rightVol){
    left_velocity=leftVel;
    right_velocity=rightVel;
    left_acceleration=leftAccel;
    right_acceleration=rightAccel;
    left_feedforward_voltage=leftVol;
    right_feedforward_voltage=rightVol;
}

void DriveMotionPlanner::Output::flip(){
    double tmp_LV=left_velocity;
    left_velocity=-right_velocity;
    right_velocity=-tmp_LV;

    double tmp_LA=left_acceleration;
    left_acceleration=-right_acceleration;
    right_acceleration=-tmp_LA;

    double tmp_LVol=left_feedforward_voltage;
    left_feedforward_voltage=-right_feedforward_voltage;
    right_feedforward_voltage=-tmp_LVol;
}

shared_ptr<DriveMotionPlanner::Output> DriveMotionPlanner::updatePID(shared_ptr<DifferentialDrive::DriveDynamics> dynamics, shared_ptr<Pose2D> current_state){
    shared_ptr<DifferentialDrive::ChassisState> adjusted_velocity = make_shared<DifferentialDrive::ChassisState>();
    //Feedback on longitudinal error (distance)
    double kPathKX=5.0;
    double kPathKY=1.0;
    double kPathKTheta=5.0;
    adjusted_velocity->linear=dynamics->chassis_velocity->linear+kPathKX*units->inches_to_meters(mError->getTranslation()->x());
    adjusted_velocity->angular=dynamics->chassis_velocity->angular+dynamics->chassis_velocity->linear*kPathKY*units->inches_to_meters(mError->getTranslation()->y())+kPathKTheta*mError->getRotation()->getRadians();
    
    double curvature= adjusted_velocity->angular/adjusted_velocity->linear;
    if (curvature>1E99 || curvature<-1E99){
        adjusted_velocity->linear=0.0;
        adjusted_velocity->angular=dynamics->chassis_velocity->angular;
    }
    //cout<<dynamics->chassis_velocity->linear<<","<<dynamics->chassis_velocity->angular<<","<<adjusted_velocity->linear<<","<<adjusted_velocity->angular<<","<<endl;

    //Compute adjusted left and right wheel velocities
    shared_ptr<DifferentialDrive::WheelState> wheel_velocities = mModel->solveInverseKinematics(adjusted_velocity);
    double left_voltage= dynamics->voltage->left+(wheel_velocities->left-dynamics->wheel_velocity->left)/mModel->Left_transmission()->Speed_per_Volt();
    double right_voltage= dynamics->voltage->right+(wheel_velocities->right-dynamics->wheel_velocity->right)/mModel->Right_transmission()->Speed_per_Volt();

    shared_ptr<Output> tmp = make_shared<Output>(wheel_velocities->right, wheel_velocities->left, dynamics->wheel_acceleration->right, dynamics->wheel_acceleration->left,  right_voltage,left_voltage);
    return tmp;
}

shared_ptr<DriveMotionPlanner::Output> DriveMotionPlanner::updateNonlinearFeedback(shared_ptr<DifferentialDrive::DriveDynamics> dynamics, shared_ptr<Pose2D> current_state){
    //cout<<dynamics->chassis_velocity->linear<<","<<dynamics->chassis_velocity->angular<<",";
    double kBeta= 1.5; //>0 EX. 3.5
    double kZeta=.4; // Damping coefficient, [0,1] EX. 0.7
    kBeta=frc::SmartDashboard::GetNumber("MotionPlanner kBeta", 1.5);
    kZeta=frc::SmartDashboard::GetNumber("MotionPlanner kZeta", .4);
    frc::SmartDashboard::PutNumber("Actual kBeta", kBeta);
    frc::SmartDashboard::PutNumber("Actual kZeta", kZeta);
    //compute gain parameter
    double k= 2.0*kZeta*sqrt(kBeta*dynamics->chassis_velocity->linear*dynamics->chassis_velocity->linear+dynamics->chassis_velocity->angular*dynamics->chassis_velocity->angular);
    frc::SmartDashboard::PutNumber("MotionPlanner k gain", k);
    
    //compute error components
    double angle_error_rads= mError->getRotation()->getRadians();
    double sin_x_over_x=util->epsilonEquals(angle_error_rads,0.0, 1E-2)? 1.0 : mError->getRotation()->sin()/angle_error_rads;

    adjusted_velocity= make_shared<DifferentialDrive::ChassisState>(
        dynamics->chassis_velocity->linear*mError->getRotation()->cos()+k*units->inches_to_meters(mError->getTranslation()->x()),
        dynamics->chassis_velocity->angular+k*angle_error_rads+dynamics->chassis_velocity->linear*kBeta*sin_x_over_x*units->inches_to_meters(mError->getTranslation()->y()));
    //cout<<adjusted_velocity->linear<<","<<adjusted_velocity->angular<<","<<endl;
    //compute adjusted right and left wheel velocities
    dynamics->chassis_velocity=adjusted_velocity;
    dynamics->wheel_velocity=mModel->solveInverseKinematics(adjusted_velocity);

    dynamics->chassis_acceleration->linear=mDt==0? 0.0: (dynamics->chassis_velocity->linear-prev_velocity_->linear)/mDt;

    dynamics->chassis_acceleration->angular=mDt==0? 0.0: (dynamics->chassis_velocity->angular-prev_velocity_->angular)/mDt;    

    prev_velocity_=dynamics->chassis_velocity;

    adjustedDynamics=mModel->solveInverseDynamics(dynamics->chassis_velocity, dynamics->chassis_acceleration);
    
    shared_ptr<DifferentialDrive::WheelState> feedforward_voltages= adjustedDynamics->voltage;

    shared_ptr<Output> tmp= make_shared<Output>(adjustedDynamics->wheel_velocity->right, adjustedDynamics->wheel_velocity->left, adjustedDynamics->wheel_acceleration->right, adjustedDynamics->wheel_acceleration->left, feedforward_voltages->right, feedforward_voltages->left);
    return tmp;
}

shared_ptr<DriveMotionPlanner::Output> DriveMotionPlanner::update(double timestamp, shared_ptr<Pose2D> current_state){
    if (mCurrentTrajectory->getProgress()==0.0 && Robot::util.epsilonEquals(mLastTime, 1E100)){
        mLastTime=timestamp;
        
    }
    mDt= timestamp-mLastTime;
    mLastTime=timestamp;
    frc::SmartDashboard::PutNumber("Loop Delta Time", mDt);
    shared_ptr<TrajectorySamplePoint> sample_point= mCurrentTrajectory->advance(mDt);

    mSetpoint=sample_point->state();

    if(!mCurrentTrajectory->isDone()){
        //generate feedforward voltages
        
        double velocity_m= units->inches_to_meters(mSetpoint->velocity());
        double curvature_m= units->meters_to_inches(mSetpoint->state()->getCurvature());
        double dcurvature_ds_m= units->meters_to_inches(units->meters_to_inches(mSetpoint->state()->getDCurvatureDs()));
        double acceleration_m= units->inches_to_meters(mSetpoint->acceleration());
        shared_ptr<DifferentialDrive::ChassisState> chassis_velocity= make_shared<DifferentialDrive::ChassisState>(velocity_m, velocity_m*curvature_m);
        shared_ptr<DifferentialDrive::ChassisState> chassis_acceleration= make_shared<DifferentialDrive::ChassisState>(acceleration_m, acceleration_m*curvature_m+ velocity_m*velocity_m*dcurvature_ds_m);
        
        //cout<<velocity_m<<","<<curvature_m<<","<<dcurvature_ds_m<<","<<acceleration_m<<","<<chassis_velocity->linear<<","<<chassis_velocity->angular<<","<<chassis_acceleration->linear<<","<<chassis_acceleration->angular<<","<<endl;
        
        dynamics= mModel->solveInverseDynamics(chassis_velocity, chassis_acceleration);
        initialDynamics=dynamics;
        mError= current_state->inverse()->transformBy(mSetpoint->state()->getPose()); //make_shared<Pose2D>(5.0, 1.0, 3.0);//


        if(mFollowerType==0){
            mOutput=make_shared<DriveMotionPlanner::Output>(dynamics->wheel_velocity->left, dynamics->wheel_velocity->right, dynamics->wheel_acceleration->left, dynamics->wheel_acceleration->right, dynamics->voltage->left, dynamics->voltage->right); 
            //cout<<"Feedforward only"<<endl;
        }else if(mFollowerType==1){
            mOutput= updatePID(dynamics, current_state);
            //    cout<<"PID"<<endl;
        
        }else if (mFollowerType==2){
            mOutput= updateNonlinearFeedback(dynamics, current_state);
            //    cout<<"NonlinearFeedback"<<endl;
        }
        
        else {
            mOutput= make_shared<DriveMotionPlanner::Output>();
            //cout<<"Invalid type, no output"<<endl;
        }

        // TODO tune all the stuff!!!
        //mCSVWriter->add(toPlannerCSV());

        return mOutput;
    
    }

    return make_shared<DriveMotionPlanner::Output>();
}

shared_ptr<DriveMotionPlanner::Output> DriveMotionPlanner::updatePIDTuner(double startVelocity){
    double mVelocity=Robot::units.inches_to_meters(Robot::drive->RPMToInchesPerSecond(startVelocity));
    shared_ptr<DifferentialDrive::ChassisState> chassis_velocity= make_shared<DifferentialDrive::ChassisState>(mVelocity, 0.0);
    shared_ptr<DifferentialDrive::ChassisState> chassis_acceleration= make_shared<DifferentialDrive::ChassisState>(0.0, 0.0);

    dynamics= mModel->solveInverseDynamics(chassis_velocity, chassis_acceleration);

    mOutput=make_shared<DriveMotionPlanner::Output>(dynamics->wheel_velocity->left, dynamics->wheel_velocity->right, dynamics->wheel_acceleration->left, dynamics->wheel_acceleration->right, dynamics->voltage->left, dynamics->voltage->right);    
    return mOutput;
}

bool DriveMotionPlanner::isDone(){
    return mCurrentTrajectory->isDone();
}

shared_ptr<Pose2D> DriveMotionPlanner::error(){
    return mError;
}

shared_ptr<TimedState> DriveMotionPlanner::setpoint(){
    return mSetpoint;
}

string DriveMotionPlanner::toPlannerCSV(){
    return toString(mSetpoint->t())+","+toString(mSetpoint->state()->getTranslation()->x())+","+toString(mSetpoint->state()->getTranslation()->y())+","+toString(mSetpoint->state()->getRotation()->getDegrees())+","+toString(mSetpoint->velocity())+","+toString(mSetpoint->acceleration())+","+toString(mSetpoint->state()->getCurvature())+","+toString(mSetpoint->state()->getDCurvatureDs())+","+toString(mError->getTranslation()->x())+","+toString(mError->getTranslation()->y())+","+toString(mError->getRotation()->getDegrees())+","+toString(initialDynamics->chassis_velocity->linear)+","+toString(initialDynamics->chassis_velocity->angular)+","+toString(adjustedDynamics->chassis_velocity->linear)+","+toString(adjustedDynamics->chassis_velocity->angular)+","+toString(initialDynamics->voltage->left)+","+toString(initialDynamics->voltage->right)+","+toString(adjustedDynamics->voltage->left)+","+toString(adjustedDynamics->voltage->right)+","+toString(Robot::drive->leftAppliedOutput())+","+toString(Robot::drive->rightAppliedOutput())+","+ Robot::robotState.toPlannerCSV()+toString(Robot::drive->mPeriodicIO->left_demand)+ ","+toString(Robot::drive->mPeriodicIO->right_demand)+","+toString(Robot::drive->getLeftVelocityNativeUnits())+","+toString(Robot::drive->getRightVelocityNativeUnits());
}