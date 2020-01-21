#include "gtest/gtest.h"

#include "lib/Util/Units.h"
#include "Subsystems/Drive.h"
#include "Robot.h"

#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
using namespace std;

class UnitConversionTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;

    shared_ptr<Subsystems::Drive> drive = make_shared<Subsystems::Drive>();
    shared_ptr<Units> units= make_shared<Units>();

    virtual void SetUp(){
        
    }
    virtual void TearDown(){}

};

TEST_F(UnitConversionTest, TestInchesToMeters){
    EXPECT_NEAR(0.0, units->inches_to_meters(0.0), kEpsilon);
    EXPECT_NEAR(2.43078, units->inches_to_meters(95.7), kEpsilon);
    EXPECT_NEAR(.254, units->inches_to_meters(10.0), kEpsilon);
    EXPECT_NEAR(1.301496, units->inches_to_meters(51.24), kEpsilon);

    EXPECT_NEAR(-2.43078, units->inches_to_meters(-95.7), kEpsilon);
    EXPECT_NEAR(-.254, units->inches_to_meters(-10.0), kEpsilon);
    EXPECT_NEAR(-1.301496, units->inches_to_meters(-51.24), kEpsilon);
}

TEST_F(UnitConversionTest, TestMetersToInches){
    EXPECT_NEAR(0.0, units->meters_to_inches(0.0), kEpsilon);
    EXPECT_NEAR(95.7, units->meters_to_inches(2.43078), kEpsilon);
    EXPECT_NEAR(10.0, units->meters_to_inches(.254), kEpsilon);
    EXPECT_NEAR(51.24, units->meters_to_inches(1.301496), kEpsilon);

    EXPECT_NEAR(-95.7, units->meters_to_inches(-2.43078), kEpsilon);
    EXPECT_NEAR(-10.0, units->meters_to_inches(-.254), kEpsilon);
    EXPECT_NEAR(-51.24, units->meters_to_inches(-1.301496), kEpsilon);
}

TEST_F(UnitConversionTest, TestIPSToRadsPerSec){
    EXPECT_NEAR(0.0/(Robot::constants.kDriveWheelRadiusInches), drive->IPSToRadiansPerSecond(0.0), kEpsilon);
    EXPECT_NEAR(15.6/(Robot::constants.kDriveWheelRadiusInches), drive->IPSToRadiansPerSecond(15.6), kEpsilon);
    EXPECT_NEAR(90.7/(Robot::constants.kDriveWheelRadiusInches), drive->IPSToRadiansPerSecond(90.7), kEpsilon);
    EXPECT_NEAR(32.5/(Robot::constants.kDriveWheelRadiusInches), drive->IPSToRadiansPerSecond(32.5), kEpsilon);

    
    EXPECT_NEAR(-15.6/(Robot::constants.kDriveWheelRadiusInches), drive->IPSToRadiansPerSecond(-15.6), kEpsilon);
    EXPECT_NEAR(-90.7/(Robot::constants.kDriveWheelRadiusInches), drive->IPSToRadiansPerSecond(-90.7), kEpsilon);
    EXPECT_NEAR(-32.5/(Robot::constants.kDriveWheelRadiusInches), drive->IPSToRadiansPerSecond(-32.5), kEpsilon);
}

TEST_F(UnitConversionTest, TestRotationsToInches){
    EXPECT_NEAR(0.0*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->rotationsToInches(0.0), kEpsilon);
    EXPECT_NEAR(32.0*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->rotationsToInches(32.0), kEpsilon);
    EXPECT_NEAR(15.6*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->rotationsToInches(15.6), kEpsilon);
    EXPECT_NEAR(107.8*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->rotationsToInches(107.8), kEpsilon);

    EXPECT_NEAR(-32.0*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->rotationsToInches(-32.0), kEpsilon);
    EXPECT_NEAR(-15.6*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->rotationsToInches(-15.6), kEpsilon);
    EXPECT_NEAR(-107.8*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->rotationsToInches(-107.8), kEpsilon);
}

TEST_F(UnitConversionTest, TestRPMToIPS){
    EXPECT_NEAR(0.0*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)/60.0, drive->RPMToInchesPerSecond(0.0), kEpsilon);
    EXPECT_NEAR(32.0*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)/60.0, drive->RPMToInchesPerSecond(32.0), kEpsilon);
    EXPECT_NEAR(15.6*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)/60.0, drive->RPMToInchesPerSecond(15.6), kEpsilon);
    EXPECT_NEAR(107.8*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)/60.0, drive->RPMToInchesPerSecond(107.8), kEpsilon);

    EXPECT_NEAR(-32.0*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)/60.0, drive->RPMToInchesPerSecond(-32.0), kEpsilon);
    EXPECT_NEAR(-15.6*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)/60.0, drive->RPMToInchesPerSecond(-15.6), kEpsilon);
    EXPECT_NEAR(-107.8*(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)/60.0, drive->RPMToInchesPerSecond(-107.8), kEpsilon);
}

TEST_F(UnitConversionTest, TestInchesToRotations){
    EXPECT_NEAR(0.0/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->inchesToRotations(0.0), kEpsilon);
    EXPECT_NEAR(32.0/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->inchesToRotations(32.0), kEpsilon);
    EXPECT_NEAR(15.6/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->inchesToRotations(15.6), kEpsilon);
    EXPECT_NEAR(107.8/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->inchesToRotations(107.8), kEpsilon);

    EXPECT_NEAR(-32.0/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->inchesToRotations(-32.0), kEpsilon);
    EXPECT_NEAR(-15.6/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->inchesToRotations(-15.6), kEpsilon);
    EXPECT_NEAR(-107.8/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR), drive->inchesToRotations(-107.8), kEpsilon);
}

TEST_F(UnitConversionTest, TestIPSToRPM){
    EXPECT_NEAR(0.0/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->inchesPerSecondToRPM(0.0), kEpsilon);
    EXPECT_NEAR(32.0/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->inchesPerSecondToRPM(32.0), kEpsilon);
    EXPECT_NEAR(15.6/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->inchesPerSecondToRPM(15.6), kEpsilon);
    EXPECT_NEAR(107.8/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->inchesPerSecondToRPM(107.8), kEpsilon);

    EXPECT_NEAR(-32.0/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->inchesPerSecondToRPM(-32.0), kEpsilon);
    EXPECT_NEAR(-15.6/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->inchesPerSecondToRPM(-15.6), kEpsilon);
    EXPECT_NEAR(-107.8/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->inchesPerSecondToRPM(-107.8), kEpsilon);
    
}

TEST_F(UnitConversionTest, TestRadsPerSecToRPM){
    EXPECT_NEAR(0.0*Robot::constants.kDriveWheelRadiusInches/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->radiansPerSecondToRPM(0.0), kEpsilon);
    EXPECT_NEAR(32.0*Robot::constants.kDriveWheelRadiusInches/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->radiansPerSecondToRPM(32.0), kEpsilon);
    EXPECT_NEAR(15.6*Robot::constants.kDriveWheelRadiusInches/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->radiansPerSecondToRPM(15.6), kEpsilon);
    EXPECT_NEAR(107.8*Robot::constants.kDriveWheelRadiusInches/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->radiansPerSecondToRPM(107.8), kEpsilon);

    EXPECT_NEAR(-32.0*Robot::constants.kDriveWheelRadiusInches/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->radiansPerSecondToRPM(-32.0), kEpsilon);
    EXPECT_NEAR(-15.6*Robot::constants.kDriveWheelRadiusInches/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->radiansPerSecondToRPM(-15.6), kEpsilon);
    EXPECT_NEAR(-107.8*Robot::constants.kDriveWheelRadiusInches/(drive->mIsHighGear? Robot::constants.kDriveHighGearIPR: Robot::constants.kDriveLowGearIPR)*60.0, drive->radiansPerSecondToRPM(-107.8), kEpsilon);
    
}