/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/DriveSignal.h"

DriveSignal::DriveSignal() {
    mLeftMotor=0.0;
    mRightMotor=0.0;
    mBrakeMode=false;
    NEUTRAL= make_shared<DriveSignal>(0.0, 0.0);
    BRAKE= make_shared<DriveSignal>(0.0, 0.0, true);
}

DriveSignal::DriveSignal(double LeftMotor, double RightMotor, bool Brake) {
    mLeftMotor=LeftMotor;
    mRightMotor=RightMotor;
    mBrakeMode=Brake;
}

DriveSignal::DriveSignal(double LeftMotor, double RightMotor){
    mLeftMotor=LeftMotor;
    mRightMotor=RightMotor;
    mBrakeMode=false;
}

double DriveSignal::getLeft(){
    return mLeftMotor;
}

double DriveSignal::getRight(){
    return mRightMotor;
}

bool DriveSignal::getBrakeMode(){
    return mBrakeMode;
}