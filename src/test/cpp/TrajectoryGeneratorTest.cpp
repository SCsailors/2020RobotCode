#pragma once

#include "gtest/gtest.h"

#include "Paths/TrajectoryGenerator.h"
#include "lib/Trajectory/TrajectoryUtil.h"

#include <vector>
#include <memory>
#include <iostream>
using namespace std;

class TrajectoryGeneratorTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
    shared_ptr<TrajectoryGenerator> generator= make_shared<TrajectoryGenerator>();
    vector<shared_ptr<TimedState>> mLeft;
    vector<shared_ptr<TimedState>> mRight;

    vector<shared_ptr<TimedState>> tmp_right;
    shared_ptr<TrajectoryUtil> traj=make_shared<TrajectoryUtil>();
};

TEST_F(TrajectoryGeneratorTest, TestGeneration){
    generator->generateTrajectories();
    mLeft=generator->getTrajectorySet()->DriveForwardTest->get(true);
    mRight=generator->getTrajectorySet()->DriveForwardTest->get(false);
    cout<<mLeft.size()<<endl;
    EXPECT_EQ(mLeft.size(), mRight.size());

    for (auto point:mLeft){
    //    cout<<point->t()<<","<<point->velocity()<<","<<point->acceleration()<<","<<point->state()->getTranslation()->x()<<","<<point->state()->getTranslation()->y()<<","<<point->state()->getRotation()->getDegrees()<<","<<endl;
    }
    tmp_right=traj->mirrorTimed(mLeft);

    for (int i = 0; i<mLeft.size(); i++){
        EXPECT_NEAR(tmp_right.at(i)->t(), mRight.at(i)->t(), kEpsilon);
        EXPECT_NEAR(tmp_right.at(i)->acceleration(), mRight.at(i)->acceleration(), kEpsilon);
        EXPECT_NEAR(tmp_right.at(i)->velocity(), mRight.at(i)->velocity(), kEpsilon);
        EXPECT_NEAR(tmp_right.at(i)->state()->getCurvature(), mRight.at(i)->state()->getCurvature(), kEpsilon);
        EXPECT_NEAR(tmp_right.at(i)->state()->getDCurvatureDs(), mRight.at(i)->state()->getDCurvatureDs(), kEpsilon);
        EXPECT_NEAR(tmp_right.at(i)->state()->getTranslation()->x(), mRight.at(i)->state()->getTranslation()->x(), kEpsilon);
        EXPECT_NEAR(tmp_right.at(i)->state()->getTranslation()->y(), mRight.at(i)->state()->getTranslation()->y(), kEpsilon);
        EXPECT_NEAR(tmp_right.at(i)->state()->getRotation()->getDegrees(), mRight.at(i)->state()->getRotation()->getDegrees(), kEpsilon);
    }

}
