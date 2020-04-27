#include "gtest/gtest.h"

#include "lib/Physics/DriveCharacterization.h"

#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
using namespace std;

class DriveCharacterizationTest: public ::testing::Test{
    protected:
    shared_ptr<DriveCharacterization> DC= make_shared<DriveCharacterization>();
    
    shared_ptr<DriveCharacterization::VelocityDataPoint> p1 = make_shared<DriveCharacterization::VelocityDataPoint>(0.0, 0.0);
    shared_ptr<DriveCharacterization::VelocityDataPoint> p2 = make_shared<DriveCharacterization::VelocityDataPoint>(12.0, 3.0);
    shared_ptr<DriveCharacterization::VelocityDataPoint> p3 = make_shared<DriveCharacterization::VelocityDataPoint>(4.0, 2.0);
    shared_ptr<DriveCharacterization::VelocityDataPoint> p4 = make_shared<DriveCharacterization::VelocityDataPoint>(16.0, 5.0);
    shared_ptr<DriveCharacterization::VelocityDataPoint> p5 = make_shared<DriveCharacterization::VelocityDataPoint>(20.0, 8.0);

    shared_ptr<DriveCharacterization::AccelerationDataPoint> a1 = make_shared<DriveCharacterization::AccelerationDataPoint>(12.0, 3.0, 2.5);
    shared_ptr<DriveCharacterization::AccelerationDataPoint> a2 = make_shared<DriveCharacterization::AccelerationDataPoint>(14.0, 4.0, 2);
    shared_ptr<DriveCharacterization::AccelerationDataPoint> a3 = make_shared<DriveCharacterization::AccelerationDataPoint>(16.0, 4.50, 2.2);
    shared_ptr<DriveCharacterization::AccelerationDataPoint> a4 = make_shared<DriveCharacterization::AccelerationDataPoint>(19.0, 7.0, 2.6);
    shared_ptr<DriveCharacterization::AccelerationDataPoint> a5 = make_shared<DriveCharacterization::AccelerationDataPoint>(23.0, 8.50, 3.0);
    
    vector<shared_ptr<DriveCharacterization::VelocityDataPoint>> VelData{p1, p2, p3, p4, p5};
    vector<shared_ptr<DriveCharacterization::AccelerationDataPoint>> AccelData{a1, a2, a3, a4, a5};
    
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
    


};

TEST_F(DriveCharacterizationTest, TestDriveCharacterization){
    shared_ptr<DriveCharacterization::CharacterizationConstants> constants=DC->characterizeDrive(VelData, AccelData);
    cout<<constants->ks<<", "<<constants->kv<<", "<<constants->ka<<", "<<endl;
}