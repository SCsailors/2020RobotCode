/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/CollectAccelerationData.h"



CollectAccelerationData::CollectAccelerationData(vector<shared_ptr<DriveCharacterization::AccelerationDataPoint>> data, bool highGear, bool reverse, bool turn) {
    mAccelerationData=data;
    mReverse=reverse;
    mHighGear=highGear;
    mTurn=turn;
    mCSVwriter=make_shared<CSVWriter>("Accel_Data", getFields());
}

void CollectAccelerationData::start(){
    Robot::drive->setHighGear(mHighGear);
    shared_ptr<DriveSignal> signal= make_shared<DriveSignal>((mReverse? -1.0:1.0)*kPower, (mReverse? -1.0:1.0)*(mTurn? -1.0:1.0)*kPower);
    Robot::drive->setOpenLoop(signal);
    mStartTime=frc::Timer::GetFPGATimestamp();
    mPrevTime=mStartTime;
    cout<<"Starting Acceleration data collection"<<endl;
    //cout<<getFields()<<endl;
}

void CollectAccelerationData::update(){
    currentVelocity= (fabs(Robot::drive->getLeftRadsPerSec()+Robot::drive->getRightRadsPerSec())/2.0);
    double currentTime= frc::Timer::GetFPGATimestamp();
    if (mPrevTime==mStartTime){
        mPrevTime=currentTime;
        mPrevVelocity=currentVelocity;
        return;
    }
    acceleration= (currentVelocity-mPrevVelocity)/(currentTime-mPrevTime);

    //ignore small accelerations
    if (acceleration<Robot::util.kEpsilon){
        mPrevTime=currentTime;
        mPrevVelocity=currentVelocity;
        return;
    }
    //cout<<toCSV()<<endl;
    
    shared_ptr<DriveCharacterization::AccelerationDataPoint> mAccelData = make_shared<DriveCharacterization::AccelerationDataPoint>(currentVelocity, kPower*12.0, acceleration);
    mAccelerationData.push_back(mAccelData);
    mCSVwriter->add(toCSV());
    mPrevTime=currentTime;
    mPrevVelocity=currentVelocity;
}

void CollectAccelerationData::done(){
    shared_ptr<DriveSignal> signal= make_shared<DriveSignal>();
    
    Robot::drive->setOpenLoop(signal);
    //mCSVwriter->close();
    cout<<"Ending Acceleration data collection"<<endl;
}

bool CollectAccelerationData::isFinished(){
    bool finished=(frc::Timer::GetFPGATimestamp()-mStartTime)>kTotalTime;

    return finished;
}

template<typename T>
string CollectAccelerationData::toString(T value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}

string CollectAccelerationData::getFields(){
    return "Velocity, Voltage, Acceleration";
}

string CollectAccelerationData::toCSV(){
    return toString(currentVelocity)+","+ toString(kPower*12.0)+","+ toString(acceleration);
}