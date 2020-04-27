/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/PIDTuner.h"
#include "Subsystems/FalconDrive.h"
#include "Subsystems/Drive.h"
#include "lib/Util/DriveSignal.h"
#include "Planners/DriveMotionPlanner.h"

#include "frc/DriverStation.h"

#include <iostream>


PIDTuner::PIDTuner(double percentPower, double duration) {
    mVelocity=percentPower*maxNeoRPM;
    mDuration=duration;
    mVelocityIPS= Subsystems::FalconDrive::getInstance()->RPMToInchesPerSecond(mVelocity);
    frc::SmartDashboard::PutNumber("PIDTuner/Velocity (Inches/second)",mVelocityIPS);
    
    std::cout<<"PIDTuner constructed"<<endl;
    Subsystems::FalconDrive::getInstance()->setPIDTuner(true);

}

PIDTuner::PIDTuner(double percentPower, double endPower, double duration){
    mVelocity=percentPower*maxNeoRPM;
    mDuration=duration;
    mEndVelocity=endPower*maxNeoRPM;

    mVelocityIPS= Subsystems::FalconDrive::getInstance()->RPMToInchesPerSecond(mVelocity);
    frc::SmartDashboard::PutNumber("PIDTuner/Velocity (Inches/second)",mVelocityIPS);
    
    std::cout<<"PIDTuner constructed"<<endl;
    Subsystems::FalconDrive::getInstance()->setPIDTuner(true);
    
}

void PIDTuner::start(){
    std::cout<<"PIDTuner starting"<<endl;
    mStartTime=frc::Timer::GetFPGATimestamp();
    shared_ptr<DriveMotionPlanner::Output> output = Subsystems::FalconDrive::getInstance()->mMotionPlanner->updatePIDTuner(mVelocity);

    frc::SmartDashboard::PutNumber("Left Feedforward", output->left_feedforward_voltage);
    frc::SmartDashboard::PutNumber("Left Demand", Subsystems::FalconDrive::getInstance()->radiansPerSecondToTicksPer100ms(output->left_velocity));
    frc::SmartDashboard::PutNumber("Left Acceleration", output->left_acceleration);

    shared_ptr<DriveSignal> signal = make_shared<DriveSignal>(Subsystems::FalconDrive::getInstance()->radiansPerSecondToTicksPer100ms(output->left_velocity), Subsystems::FalconDrive::getInstance()->radiansPerSecondToTicksPer100ms(output->right_velocity));
    shared_ptr<DriveSignal> feedforward= make_shared<DriveSignal>(output->left_feedforward_voltage / 12.0, output->right_feedforward_voltage /12.0);
    //shared_ptr<DriveSignal> feedforward=make_shared<DriveSignal>(2.0, 2.0);
    Subsystems::FalconDrive::getInstance()->setVelocity(signal, feedforward);    
    
    std::cout<<"PIDTuner started"<<endl;
}

void PIDTuner::update(){
    frc::SmartDashboard::PutNumber("PID Tuning Error", Robot::util.limit((Subsystems::FalconDrive::getInstance()->getLinearVelocity()-mVelocityIPS), 2.0));
    
}

void PIDTuner::done(){
    shared_ptr<DriveMotionPlanner::Output> output = Subsystems::FalconDrive::getInstance()->mMotionPlanner->updatePIDTuner(mEndVelocity);

    shared_ptr<DriveSignal> signal = make_shared<DriveSignal>(Subsystems::FalconDrive::getInstance()->radiansPerSecondToTicksPer100ms(output->left_velocity), Subsystems::FalconDrive::getInstance()->radiansPerSecondToTicksPer100ms(output->right_velocity));
    shared_ptr<DriveSignal> feedforward= make_shared<DriveSignal>(output->left_feedforward_voltage / 12.0, output->right_feedforward_voltage / 12.0);

    Subsystems::FalconDrive::getInstance()->setVelocity(signal, feedforward);
    
    std::cout<<"PIDTuner done"<<endl;
}

bool PIDTuner::isFinished(){
    return (frc::Timer::GetFPGATimestamp()-mStartTime)>mDuration;
}

