#include "gtest/gtest.h"

#include "RobotState.h"

#include <memory>
#include <iostream>
//using namespace std;

class Robot_StateTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}

    //shared_ptr<RobotState> r=make_shared<RobotState>();
    //std::cout << r->vehicle
};