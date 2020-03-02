/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/CollectVelocityData.h"
#include "Subsystems/FalconDrive.h"

CollectVelocityData::CollectVelocityData(vector<shared_ptr<DriveCharacterization::VelocityDataPoint>> data, bool highgear, bool reverse, bool turn) {
    mVelocityData=data;
    mHighGear=highgear;
    mTurn=turn;
    mCSVWriter=make_shared<CSVWriter>("Velocity_Data", getFields());
}

bool CollectVelocityData::isFinished(){
    return Finished;
}

void CollectVelocityData::start(){
    Subsystems::FalconDrive::getInstance()->setHighGear(mHighGear);
    mStartTime=frc::Timer::GetFPGATimestamp();
    cout<<"Starting velocity data collection"<<endl;
}

void CollectVelocityData::update(){
    
    if(mPercentPower>kMaxPower){
        
        
        Finished=true;
        return;
    }
    mPercentPower= kRampRate*(frc::Timer::GetFPGATimestamp()-mStartTime);
    
    Subsystems::FalconDrive::getInstance()->setOpenLoop(make_shared<DriveSignal>((mReverse? -1.0:1.0)*mPercentPower, (mReverse? -1.0:1.0)*(mTurn? -1.0:1.0)*mPercentPower));
    velocity= (fabs(Subsystems::FalconDrive::getInstance()->getLeftRadsPerSec())+fabs(Subsystems::FalconDrive::getInstance()->getRightRadsPerSec()))/2.0; //now in radians per second of drive wheels
    voltage= mPercentPower*12.0;//volts
    shared_ptr<DriveCharacterization::VelocityDataPoint> VelDataPoint= make_shared<DriveCharacterization::VelocityDataPoint>(velocity, voltage);
    mVelocityData.push_back(VelDataPoint);
    mCSVWriter->add(toCSV());
}

void CollectVelocityData::done(){
    
    Subsystems::FalconDrive::getInstance()->setOpenLoop(make_shared<DriveSignal>(0,0));
    //mCSVWriter->close();
    cout<<"Ending velocity data collection"<<endl;
}

template<typename T>
string CollectVelocityData::toString(T value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}

string CollectVelocityData::toCSV(){
    return toString(velocity)+","+toString(voltage);
}

string CollectVelocityData::getFields(){
    return "Velocity, Voltage";
}
