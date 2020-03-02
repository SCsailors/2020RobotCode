/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/CollectCurvatureData.h"
#include "Subsystems/FalconDrive.h"

#include <RobotState.h>

CollectCurvatureData::CollectCurvatureData(vector<shared_ptr<DriveCharacterization::CurvatureDataPoint>> data, bool highGear, bool reverse) {
    mCurvatureData=data;
    mHighGear=highGear;
    mReverse=reverse;
    mCSVWriter=make_shared<CSVWriter>("Curvature_Data", getFields());
}

void CollectCurvatureData::start(){
    Subsystems::FalconDrive::getInstance()->setHighGear(mHighGear);

    Subsystems::FalconDrive::getInstance()->setOpenLoop(make_shared<DriveSignal>(kStartPower, kStartPower));
    mStartTime= frc::Timer::GetFPGATimestamp();
}

void CollectCurvatureData::update(){
    double t= frc::Timer::GetFPGATimestamp();
    if(t<kStartTime){
        return;
    }
    rightPower= kStartPower+ (t-kStartPower)*kRampRate;
    if(rightPower>kMaxPower){
        Finished=true;
        return;
    }
    
    Subsystems::FalconDrive::getInstance()->setOpenLoop(make_shared<DriveSignal>((mReverse? -1.0:1.0)*kStartPower, (mReverse?-1.0:1.0)*rightPower));

    shared_ptr<DriveCharacterization::CurvatureDataPoint> CurvData = make_shared<DriveCharacterization::CurvatureDataPoint>(FRC_7054::RobotState::getInstance()->getPredictedVelocity()->dx,
        FRC_7054::RobotState::getInstance()->getPredictedVelocity()->dtheta, kStartPower*12.0, rightPower*12.0);
    mCurvatureData.push_back(CurvData);

    mCSVWriter->add(toCSV());
}

bool CollectCurvatureData::isFinished(){
    return Finished;
}

void CollectCurvatureData::done(){
    shared_ptr<DriveSignal> signal = make_shared<DriveSignal>();
    Subsystems::FalconDrive::getInstance()->setOpenLoop(signal->BRAKE);
    mCSVWriter->close();
}

template<typename T>
string CollectCurvatureData::toString(T value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}

string CollectCurvatureData::getFields(){
    return "LinearVelocity, AngularVelocity, LeftVoltage, RightVoltage";
}

string CollectCurvatureData::toCSV(){
    return toString(FRC_7054::RobotState::getInstance()->getPredictedVelocity()->dx)+ ","+toString(FRC_7054::RobotState::getInstance()->getPredictedVelocity()->dtheta)+","+ toString(kStartPower*12.0)+","+toString(rightPower*12.0);
}