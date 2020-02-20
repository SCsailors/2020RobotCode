/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include "Subsystems/Drive.h"
#include <cmath>
#include "Constants.h"

#include <RobotState.h>

using namespace Subsystems;
namespace Subsystems{
Drive::PeriodicIO::PeriodicIO(){}


Drive::Drive() {
    kP=Constants::kDriveLowGearVelocityKp;//1E-4*.5;//PID Tuning: 1E-4*.5; Trajectory: 8e-5;
    kI=Constants::kDriveLowGearVelocityKi;//1.864E-6*.35;//PID Tuning: 1.864E-6*.35; Trajectory: 0.0;
    kD=Constants::kDriveLowGearVelocityKd;//0.00025;//1.20713E-6*4;//PID Tuning: 1.20713E-6*4; Trajectory: 0.0;
    kFF=Constants::kDriveLowGearVelocityKf;//.000015;//PID Tuning and Trajectory: .000015;
    kIZ=Constants::kDriveLowGearVelocityKiZone;
    kA=Constants::kAcceleration;
    
    frc::SmartDashboard::PutNumber("Drive P", kP);
    frc::SmartDashboard::PutNumber("Drive I", kI);
    frc::SmartDashboard::PutNumber("Drive D", kD);
    frc::SmartDashboard::PutNumber("Drive IZone", kIZ);
    frc::SmartDashboard::PutNumber("Drive FF", kFF);
    frc::SmartDashboard::PutNumber("Drive Acceleration", kA);
    
    //Initialize hardware here wrap all spark and navx in #ifdef CompetitionBot / #endif because there aren't binaries for on workstation unit testing
    #ifdef CompetitionBot
    configureSparkMaxPID();    
     leftMaster.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
     rightMaster.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
     
     leftMaster.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 20);
     rightMaster.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 20);

    
     leftMaster.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 5);
     rightMaster.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 5);
     
     leftSlave1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10000);
     rightSlave1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10000);

     
     leftSlave1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10000);
     rightSlave1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10000);

     
     leftSlave1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10000);
     rightSlave1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10000);

     
     leftSlave2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10000);
     rightSlave2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10000);

     
     leftSlave2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10000);
     rightSlave2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10000);

     
     leftSlave2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10000);
     rightSlave2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10000);
     
     
     
     leftMaster.SetOpenLoopRampRate(1.0);
     rightMaster.SetOpenLoopRampRate(1.0);
     leftMaster.SetClosedLoopRampRate(0.001);
     rightMaster.SetClosedLoopRampRate(0.001);
     leftMaster.SetInverted(true);
    
     leftMaster.EnableVoltageCompensation(12.0);
     rightMaster.EnableVoltageCompensation(12.0);
     leftSlave1.EnableVoltageCompensation(12.0);
     leftSlave2.EnableVoltageCompensation(12.0);

     rightSlave1.EnableVoltageCompensation(12.0);
     rightSlave2.EnableVoltageCompensation(12.0);

     //rightMaster.SetInverted(true);
     leftSlave1.Follow(leftMaster, false);
     rightSlave1.Follow(rightMaster, false);



      leftSlave2.Follow(leftMaster, false);
     rightSlave2.Follow(rightMaster, false);
    
    
     
    #endif


    mIsHighGear=true;
    setHighGear(false);

    setOpenLoop(mDriveSignal->NEUTRAL);

    mIsBrakeMode=true;
    setBrakeMode(false);

    mMotionPlanner=make_shared<DriveMotionPlanner>();

}

#ifdef CompetitionBot
void Drive::configureSparkMaxPID(){
    leftMaster.RestoreFactoryDefaults();
    rightMaster.RestoreFactoryDefaults();
    /*
    leftMasterPID.SetP(Constants::kDriveLowGearVelocityKp);
    leftMasterPID.SetI(Constants::kDriveLowGearVelocityKi);
    leftMasterPID.SetD(Constants::kDriveLowGearVelocityKd);
    leftMasterPID.SetFF(Constants::kDriveLowGearVelocityKf);
    leftMasterPID.SetIZone(Constants::kDriveLowGearVelocityKiZone);
    leftMasterPID.SetOutputRange(-1.0, 1.0);
    
    rightMasterPID.SetP(Constants::kDriveLowGearVelocityKp);
    rightMasterPID.SetI(Constants::kDriveLowGearVelocityKi);
    rightMasterPID.SetD(Constants::kDriveLowGearVelocityKd);
    rightMasterPID.SetFF(Constants::kDriveLowGearVelocityKf);
    rightMasterPID.SetIZone(Constants::kDriveLowGearVelocityKiZone);
    rightMasterPID.SetOutputRange(-1.0, 1.0);
    */
    
   
    leftMasterPID.SetP(kP);
    leftMasterPID.SetI(kI);
    leftMasterPID.SetD(kD);
    leftMasterPID.SetFF(kFF);
    leftMasterPID.SetIZone(kIZ);
    leftMasterPID.SetIMaxAccum(10.0);
    leftMasterPID.SetOutputRange(-1.0, 1.0);
    
    rightMasterPID.SetP(kP);
    rightMasterPID.SetI(kI);//1e-6
    rightMasterPID.SetD(kD);
    rightMasterPID.SetFF(kFF);
    rightMasterPID.SetIZone(kIZ);
    rightMasterPID.SetIMaxAccum(10.0);
    rightMasterPID.SetOutputRange(-1.0, 1.0);
    
    leftSlavePID.SetP(kP);
    leftSlavePID.SetI(kI);
    leftSlavePID.SetD(kD);
    leftSlavePID.SetFF(kFF);
    leftSlavePID.SetIZone(kIZ);
    leftSlavePID.SetIMaxAccum(10.0);
    leftSlavePID.SetOutputRange(-1.0, 1.0);
    
    leftSlavePID.SetP(kP);
    leftSlavePID.SetI(kI);//1e-6
    leftSlavePID.SetD(kD);
    leftSlavePID.SetFF(kFF);
    leftSlavePID.SetIZone(kIZ);
    leftSlavePID.SetIMaxAccum(10.0);
    leftSlavePID.SetOutputRange(-1.0, 1.0);

    leftSlavePID2.SetP(kP);
    leftSlavePID2.SetI(kI);
    leftSlavePID2.SetD(kD);
    leftSlavePID2.SetFF(kFF);
    leftSlavePID2.SetIZone(kIZ);
    leftSlavePID2.SetIMaxAccum(10.0);
    leftSlavePID2.SetOutputRange(-1.0, 1.0);
    
    leftSlavePID2.SetP(kP);
    leftSlavePID2.SetI(kI);//1e-6
    leftSlavePID2.SetD(kD);
    leftSlavePID2.SetFF(kFF);
    leftSlavePID2.SetIZone(kIZ);
    leftSlavePID2.SetIMaxAccum(10.0);
    leftSlavePID2.SetOutputRange(-1.0, 1.0);

}

#endif


void Drive::OnStart(double timestamp){
    //shared_ptr<DriveSignal> Signal= make_shared<DriveSignal>(0.0, 0.0);
    //setOpenLoop(Signal);
    setBrakeMode(false);
    //startLogging();
}

void Drive::OnLoop(double timestamp){
    
    //if (PIDTuning){
    #ifdef CompetitionBot
    
    frc::SmartDashboard::PutNumber("Actual Drive P", leftMasterPID.GetP());
    frc::SmartDashboard::PutNumber("Actual Drive I", leftMasterPID.GetI());
    frc::SmartDashboard::PutNumber("Actual Drive D", leftMasterPID.GetD());
    frc::SmartDashboard::PutNumber("Actual Drive IZone", leftMasterPID.GetIZone());
    frc::SmartDashboard::PutNumber("Actual Drive FF", leftMasterPID.GetFF());
    frc::SmartDashboard::PutNumber("Actual Drive kA", kA);
    
    frc::SmartDashboard::PutNumber("Actual Drive P Slave", leftSlavePID.GetP());
    frc::SmartDashboard::PutNumber("Actual Drive I Slave", leftSlavePID.GetI());
    frc::SmartDashboard::PutNumber("Actual Drive D Slave", leftSlavePID.GetD());
    frc::SmartDashboard::PutNumber("Actual Drive IZone Slave", leftSlavePID.GetIZone());
    frc::SmartDashboard::PutNumber("Actual Drive FF Slave", leftSlavePID.GetFF());
    
    
    #endif
    //}
    if (mDriveControlState==0){

    } else if (mDriveControlState==1){
        if(!PIDTuning){
            updatePathFollower();
        }
        
    }

    if(mAutoShift){
        //handleAutoShift();
    } else{
        //setHighGear(false);
    }
}

void Drive::OnStop(double timestamp){
    stop();
    //stopLogging();
}

double Drive::IPSToRadiansPerSecond(double inches_per_second){ //vel/wheel radius
    return inches_per_second/Constants::kDriveWheelRadiusInches;
}//Inches/Sec->Radians/Sec of drive wheels.

double Drive::rotationsToInches(double rotations){
    return rotations* (mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR);
}

double Drive::RPMToInchesPerSecond(double rpm){
    return rotationsToInches(rpm)/60.0; 
}

double Drive::inchesToRotations(double inches){
    return inches/(mIsHighGear? Constants::kDriveHighGearIPR: Constants::kDriveLowGearIPR); 
}

double Drive::inchesPerSecondToRPM(double inches_per_second){
    return inchesToRotations(inches_per_second)*60.0; 
}

double Drive::radiansPerSecondToRPM(double rad_s){ //rad/s of Drive wheels to RPM of the drive Encoders
    return inchesPerSecondToRPM(rad_s*Constants::kDriveWheelRadiusInches);
} //rad/s->inches/sec (both of drive wheels)->RPM (of Encoders)

void Drive::setOpenLoop(shared_ptr<DriveSignal> signal){
    if(mDriveControlState!=Open_Loop){
        setBrakeMode(false);
        mAutoShift=true;
        mDriveControlState=Open_Loop;

    }
    mPeriodicIO->left_demand= signal->getLeft();
    mPeriodicIO->right_demand= signal->getRight();
    mPeriodicIO->left_feedforward=0.0;
    mPeriodicIO->right_feedforward=0.0;
}

void Drive::setVelocity(shared_ptr<DriveSignal> signal, shared_ptr<DriveSignal> feedforward){
    if(mDriveControlState!= Path_Following){
        //entering velocity controlled state
        setBrakeMode(true);
        mAutoShift=false;

        mDriveControlState= Path_Following;
    }
    mPeriodicIO->left_demand= signal->getLeft();
    mPeriodicIO->right_demand= signal->getRight();
    mPeriodicIO->left_feedforward=feedforward->getLeft();
    mPeriodicIO->right_feedforward=feedforward->getRight();

}

void Drive::setTrajectory(shared_ptr<TrajectoryIterator> trajectory){
    mOverrideTrajectory= false;
    mMotionPlanner->reset();
    mMotionPlanner->setTrajectory(trajectory);
    mDriveControlState=Path_Following;
}

bool Drive::isDoneWithTrajectory(){
    if(mDriveControlState!=Path_Following){
        return false;
    }
    return mMotionPlanner->isDone() || mOverrideTrajectory;
}

bool Drive::isHighGear(){
    return mIsHighGear;
}

void Drive::setHighGear(bool wantsHighGear){
    if(wantsHighGear!= mIsHighGear){

        if(wantsHighGear){
            mIsHighGear=true;
            #ifdef CompetitionBot  
            gearSolenoid.Set(frc::DoubleSolenoid::Value::kForward); // TODO check if forward is high gear
            #endif
        } else if (!wantsHighGear) {

            mIsHighGear=false;
            #ifdef CompetitionBot  
            gearSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
            #endif
        }

    }
    frc::SmartDashboard::PutString("Drive/ Setting HighGear", mIsHighGear? "HighGear": "LowGear");
}

bool Drive::isBrakeMode(){
    return mIsBrakeMode;
}

void Drive::setBrakeMode(bool on){
    if(mIsBrakeMode!= on){
        mIsBrakeMode= on;
        
        if(mIsBrakeMode){
            #ifdef CompetitionBot
            leftMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            rightMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            leftSlave1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            rightSlave1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            #ifdef COMPETITIONBOT
            leftSlave2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            rightSlave2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            #endif
            #endif
        }else{
            #ifdef CompetitionBot
            leftMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            rightMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            leftSlave1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            rightSlave1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            #ifdef COMPETITIONBOT
            leftSlave2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            rightSlave2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            #endif
            #endif

        }
    
    }
}

shared_ptr<Rotation2D> Drive::getHeading(){
    return mPeriodicIO->gyro_heading;
}

void Drive::setHeading(shared_ptr<Rotation2D> heading){
    mPeriodicIO->gyro_heading=heading;
    #ifdef CompetitionBot
     mGyroOffset= heading->rotateBy(heading->fromDegrees(navx.GetFusedHeading())->inverse());
    #endif
    
}

void Drive::stop(){
    setOpenLoop(mDriveSignal->NEUTRAL);
}

void Drive::outputTelemetry(){
    
    frc::SmartDashboard::PutNumber("Drive/Right Encoder Distance", mPeriodicIO->right_distance);//check first four
    frc::SmartDashboard::PutNumber("Drive/Left Encoder Distance", mPeriodicIO->left_distance);
    frc::SmartDashboard::PutNumber("Drive/Right Encoder Rotations", mPeriodicIO->right_rotations);
    frc::SmartDashboard::PutNumber("Drive/Left Encoder Rotations", mPeriodicIO->left_rotations);
    frc::SmartDashboard::PutNumber("Drive/Right Linear Velocity", getRightLinearvelocity());
    frc::SmartDashboard::PutNumber("Drive/Left Linear Velocity", getLeftLinearVelocity());

    frc::SmartDashboard::PutNumber("Drive/X Error", mPeriodicIO->error->inverse()->getTranslation()->x());
    frc::SmartDashboard::PutNumber("Drive/Y Error", mPeriodicIO->error->inverse()->getTranslation()->y());
    frc::SmartDashboard::PutNumber("Drive/Theta Error", mPeriodicIO->error->inverse()->getRotation()->getDegrees());

    frc::SmartDashboard::PutBoolean("Drive/ Is High Gear", isHighGear());

    if (getHeading()   != NULL){
        frc::SmartDashboard::PutNumber("Drive/Gyro Heading", getHeading()->getDegrees());
    }
}

void Drive::resetEncoders(){
    mPeriodicIO=make_shared<Drive::PeriodicIO>();
    #ifdef CompetitionBot
     leftMasterEncoder.SetPosition(0.0);
     rightMasterEncoder.SetPosition(0.0);
    #endif
}

void Drive::zeroSensors(){
    mPeriodicIO=make_shared<PeriodicIO>();
    shared_ptr<Rotation2D> rotation= make_shared<Rotation2D>();
    setHeading(rotation);
    resetEncoders();
    mAutoShift=true;
}

double Drive::getLeftEncoderRotations(){
    return mPeriodicIO->left_rotations;
} //Rotations of Encoders

double Drive::getRightEncoderRotations(){
    return mPeriodicIO->right_rotations;
}

double Drive::getLeftEncoderDistance(){
    return rotationsToInches(getLeftEncoderRotations());
} //distance wheels travel

double Drive::getRightEncoderDistance(){
    return rotationsToInches(getRightEncoderRotations());
}

double Drive::getRightVelocityNativeUnits(){
    return mPeriodicIO->right_velocity_rpm;// of drive encoders
}

double Drive::getRightLinearvelocity(){//inches /second of drive wheels
    return RPMToInchesPerSecond(getRightVelocityNativeUnits());
}

double Drive::getLeftVelocityNativeUnits(){//rpm of encoders
    return mPeriodicIO->left_velocity_rpm;
}

double Drive::getLeftLinearVelocity(){
    return RPMToInchesPerSecond(getLeftVelocityNativeUnits());
}

double Drive::getRightRadsPerSec(){
    return IPSToRadiansPerSecond(getRightLinearvelocity());
}

double Drive::getLeftRadsPerSec(){ //rads/sec of drive wheels
    return IPSToRadiansPerSecond(getLeftLinearVelocity());
}

double Drive::getLinearVelocity(){// inches/sec of drive wheels
    return (getLeftLinearVelocity()+getRightLinearvelocity())/2.0;
}

double Drive::getAngularVelocity(){
    return (getRightLinearvelocity()-getLeftLinearVelocity())/Constants::kDriveWheelTrackWidthInches;
}

void Drive::overrideTrajectory(bool value){
    mOverrideTrajectory=value;
}

void Drive::updatePathFollower(){
    
        
    if (mDriveControlState==Path_Following){
        double now = frc::Timer::GetFPGATimestamp(); 
        shared_ptr<Pose2D> pose= FRC_7054::RobotState::getInstance()->getLatestFieldToVehicle();//TODO check units for conversion from RadiansPerSecondToRPS()
        //cout<<"Pose: "<<pose->getTranslation()->x()<<","<<pose->getTranslation()->y()<<","<<pose->getRotation()->getDegrees()<<","<<endl;
        
            output= mMotionPlanner->update(now, pose);
        
        
        mPeriodicIO->error=mMotionPlanner->mError;
        //cout<<"Error: "<<mPeriodicIO->error->getTranslation()->x()<<","<<mPeriodicIO->error->getTranslation()->y()<<","<<mPeriodicIO->error->getRotation()->getDegrees()<<endl;

        shared_ptr<DriveSignal> signal = make_shared<DriveSignal>(radiansPerSecondToRPM(output->left_velocity), radiansPerSecondToRPM(output->right_velocity));
        shared_ptr<DriveSignal> feedforward= make_shared<DriveSignal>(output->left_feedforward_voltage, output->right_feedforward_voltage);
        
        if(!mOverrideTrajectory){
            setVelocity(signal, feedforward);

            mPeriodicIO->left_accel= radiansPerSecondToRPM(output->left_acceleration);
            mPeriodicIO->right_accel= radiansPerSecondToRPM(output->right_acceleration);
        } else{
            setVelocity(mDriveSignal->BRAKE, mDriveSignal->BRAKE);
            mPeriodicIO->left_accel=mPeriodicIO->right_accel=0.0;
        }
    }
}

void Drive::handleAutoShift(){ //TODO stop from thrashing: gets up to speed, shifts, then down shifts immediately
    double linear_velocity= fabs(getLinearVelocity());
    double angular_velocity= fabs(getAngularVelocity());
    if(mIsHighGear&& linear_velocity<Constants::kDriveDownShiftVelocity){ // && angular_velocity< constants->kDriveDownShiftAngularVelocity
        setHighGear(false);
    } else if (!mIsHighGear && linear_velocity> Constants::kDriveDownShiftVelocity){
        setHighGear(true);
    }
}

void Drive::setPIDTuner(bool pidTuning){
    PIDTuning=pidTuning;
    /*
    shared_ptr<DriveSignal> signal = make_shared<DriveSignal>();
    shared_ptr<DriveSignal> feedforward = make_shared<DriveSignal>();
    if (PIDTuning){
        //mDriveControlState=Path_Following;
        
        setVelocity(signal, feedforward);
        mOverrideTrajectory=false;
    } else{
        //mDriveControlState=Open_Loop;
        
        
        setOpenLoop(signal);
        mOverrideTrajectory=true;
    }
    */
}


void Drive::readPeriodicInputs(){
    double prevLeftRotations=mPeriodicIO->left_rotations;
    double prevRightRotations=mPeriodicIO->right_rotations;
    
    #ifdef CompetitionBot
    mPeriodicIO->left_rotations= leftMasterEncoder.GetPosition(); // TODO motor controller data
    mPeriodicIO->right_rotations=rightMasterEncoder.GetPosition(); // TODO motor controller data
    mPeriodicIO->left_velocity_rpm=leftMasterEncoder.GetVelocity(); // TODO motor controller data
    mPeriodicIO->right_velocity_rpm=rightMasterEncoder.GetVelocity(); // TODO motor controller data
    
    #endif
    shared_ptr<Rotation2D> tmp =make_shared<Rotation2D>();
    #ifdef CompetitionBot
    mPeriodicIO->gyro_heading=tmp->fromDegrees( navx.GetFusedHeading())->rotateBy(mGyroOffset); 
    #endif
    #ifndef CompetitionBot
    mPeriodicIO->gyro_heading= tmp;
    #endif
    double deltaLeftRotations= ((mPeriodicIO->left_rotations-prevLeftRotations));
    mPeriodicIO->left_distance+= deltaLeftRotations*pi*Constants::kDriveWheelDiameterInches;

    double deltaRightRotations= (mPeriodicIO->right_rotations-prevRightRotations);
    mPeriodicIO->right_distance+= deltaRightRotations*pi*Constants::kDriveWheelDiameterInches;
    if(mCSVWriter!=NULL){
        mCSVWriter->add(toCSV());
    }
    
     p = frc::SmartDashboard::GetNumber("Drive P", kP);
     i = frc::SmartDashboard::GetNumber("Drive I", kI);
     d = frc::SmartDashboard::GetNumber("Drive D", kD);
     iz = frc::SmartDashboard::GetNumber("Drive IZone", kIZ);
     ff = frc::SmartDashboard::GetNumber("Drive FF", kFF);
     a = frc::SmartDashboard::GetNumber("Drive Acceleration", 4.0);
    
}
//remove from Subsystem and inherited classes
void Drive::writeToLog(){
    //done in readPeriodicInputs()
}

double Drive::leftAppliedOutput(){
    #ifdef CompetitionBot
    return leftMaster.GetAppliedOutput();
    #endif
    return 0.0;
}

double Drive::rightAppliedOutput(){
    #ifdef CompetitionBot
    return rightMaster.GetAppliedOutput();
    #endif
    return 0.0;
}

void Drive::writePeriodicOutputs(){
    #ifdef CompetitionBot
    if((p != kP)) { rightMasterPID.SetP(p); leftMasterPID.SetP(p); rightSlavePID.SetP(p); leftSlavePID.SetP(p); rightSlavePID2.SetP(p); leftSlavePID2.SetP(p); kP = p; }
    if((i != kI)) { rightMasterPID.SetI(i); leftMasterPID.SetI(i); rightSlavePID.SetI(i); leftSlavePID.SetI(i); rightSlavePID2.SetI(i); leftSlavePID2.SetI(i); kI = i; }
    if((d != kD)) { rightMasterPID.SetD(d); leftMasterPID.SetD(d);  rightSlavePID.SetD(d); leftSlavePID.SetD(d); rightSlavePID2.SetD(d); leftSlavePID2.SetD(d); kD = d; }
    if((iz != kIZ)) { rightMasterPID.SetIZone(iz); leftMasterPID.SetIZone(iz); rightSlavePID.SetIZone(iz); leftSlavePID.SetIZone(iz); rightSlavePID2.SetIZone(iz); leftSlavePID2.SetIZone(iz); kIZ = iz; }
    if((ff != kFF)) { rightMasterPID.SetFF(ff); leftMasterPID.SetFF(ff);  rightSlavePID.SetFF(ff); leftSlavePID.SetFF(ff); rightSlavePID2.SetFF(ff); leftSlavePID2.SetFF(ff); kFF = ff; }
    if((a!=kA)){ kA=a; }
    #endif
    if(mDriveControlState==Open_Loop){
        //setting percent output for open loop
        #ifdef CompetitionBot
        leftMasterPID.SetReference(mPeriodicIO->left_demand, rev::ControlType::kDutyCycle); //percent output from -1 to 1
        rightMasterPID.SetReference(mPeriodicIO->right_demand, rev::ControlType::kDutyCycle);
        

        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Percent", leftMaster.GetAppliedOutput());
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Percent", rightMaster.GetAppliedOutput());

        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Bus Voltage", leftMaster.GetBusVoltage());
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Bus Voltage", rightMaster.GetBusVoltage());
        #endif

    } else{
        
        //setting velocity (rpm) for Trajectory following
        #ifdef CompetitionBot
        //leftMasterPID.SetReference(mPeriodicIO->left_demand, rev::ControlType::kVelocity, 0, (mPeriodicIO->left_feedforward + Constants::kDriveLowGearVelocityKd*mPeriodicIO->left_accel)); 
        //rightMasterPID.SetReference(mPeriodicIO->right_demand, rev::ControlType::kVelocity, 0, mPeriodicIO->right_feedforward + Constants::kDriveLowGearVelocityKd*mPeriodicIO->right_accel);

        leftMasterPID.SetReference(mPeriodicIO->left_demand, rev::ControlType::kVelocity,0,mPeriodicIO->left_feedforward+mPeriodicIO->left_accel/(Constants::kDriveMaxAcceleration*kA)); 
        rightMasterPID.SetReference(mPeriodicIO->right_demand, rev::ControlType::kVelocity,0, mPeriodicIO->right_feedforward+mPeriodicIO->right_accel/(Constants::kDriveMaxAcceleration*kA));
        //if(mPeriodicIO->left_demand!=0.0){

        
        //cout<<mPeriodicIO->left_demand<<","<<mPeriodicIO->right_demand<<","<<mPeriodicIO->left_feedforward<<","<<mPeriodicIO->right_feedforward<<","<<leftMaster.GetAppliedOutput()<<","<<rightMaster.GetAppliedOutput()<<","<<endl;
        //}

        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left demand", mPeriodicIO->left_demand);
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right demand", mPeriodicIO->right_demand);

        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left feedforward", mPeriodicIO->left_feedforward);
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right feedforward", mPeriodicIO->right_feedforward);
        
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Percent", leftMaster.GetAppliedOutput());
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Percent", rightMaster.GetAppliedOutput());


        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Velocity", getLeftVelocityNativeUnits());
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Velocity", getRightVelocityNativeUnits());

        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Voltage", leftMaster.GetBusVoltage());
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Voltage", rightMaster.GetBusVoltage());
        
        //frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Error", Robot::util.limit(getLeftVelocityNativeUnits()-mPeriodicIO->left_demand,1000.0));
        //frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Error", Robot::util.limit(getRightVelocityNativeUnits()-mPeriodicIO->right_demand,1000.0));

        //frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Error", Robot::util.limit(getLeftVelocityNativeUnits()-mPeriodicIO->left_demand, 500.0));
        //frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Error", Robot::util.limit(getRightVelocityNativeUnits()-mPeriodicIO->right_demand,500.0));

        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Error", getLeftVelocityNativeUnits()-mPeriodicIO->left_demand);
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Error", getRightVelocityNativeUnits()-mPeriodicIO->right_demand);

        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Left Output Error Lag", getLeftVelocityNativeUnits()-prev_leftVel);
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Right Output Error Lag", getRightVelocityNativeUnits()-prev_rightVel);

       
        frc::SmartDashboard::PutNumber("Drive/DriveOutputs/ Error Distance", std::sqrt(mPeriodicIO->error->getTranslation()->x()*mPeriodicIO->error->getTranslation()->x()+mPeriodicIO->error->getTranslation()->y()*mPeriodicIO->error->getTranslation()->y()));
        #endif
        prev_leftVel=mPeriodicIO->left_demand;
        prev_rightVel=mPeriodicIO->right_demand;
    }
}

bool Drive::checkSystem(){
    return true;
}

void Drive::startLogging(){
    if(mCSVWriter==NULL){
        mCSVWriter=make_shared<CSVWriter>("Drive-Logs", getFields());
        
    }
}

void Drive::stopLogging(){
    if(mCSVWriter!=NULL){
        mCSVWriter->close();
        mCSVWriter=NULL;
    }
}

template<typename T>
string Drive::toString(T value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}

string Drive::getFields(){
    return "T, left_rotations, right_rotations,left_distance,right_distance,left_velocity_rpm,right_velocity_rpm,left_demand,right_demand,left_accel,right_accel,left_feedforward,right_feedforward, X, Y, Theta, Linear Velocity, Left Applied Output, Right Applied Output, Left Output Error, Right Output Error,";
}

string Drive::toCSV(){
    shared_ptr<Pose2D> pose = FRC_7054::RobotState::getInstance()->getLatestFieldToVehicle();
    return toString(frc::Timer::GetFPGATimestamp())+","+toString(mPeriodicIO->left_rotations)+","+toString(mPeriodicIO->right_rotations)+","+toString(mPeriodicIO->left_distance)+","+toString(mPeriodicIO->right_distance)+","+toString(mPeriodicIO->left_velocity_rpm)+","+toString(mPeriodicIO->right_velocity_rpm)+","+
    toString(mPeriodicIO->left_demand)+","+toString(mPeriodicIO->right_demand)+","+toString(mPeriodicIO->left_accel)+","+toString(mPeriodicIO->right_accel)+","+toString(mPeriodicIO->left_feedforward)+","+toString(mPeriodicIO->right_feedforward)+","+toString(pose->getTranslation()->x())+","+toString(pose->getTranslation()->y())+","+toString(pose->getRotation()->getDegrees())+","+toString(getLinearVelocity())
    #ifdef CompetitionBot
    +","+toString(leftMaster.GetAppliedOutput())+","+toString(rightMaster.GetAppliedOutput())+","+toString(mPeriodicIO->left_demand-getLeftVelocityNativeUnits())+","+toString(mPeriodicIO->right_demand-getRightVelocityNativeUnits())
    #endif
    ;
}

}

