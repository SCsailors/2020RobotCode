#include "gtest/gtest.h"

#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Geometry/Pose2D.h"

#include <cmath>
#include <memory>
using namespace std;

class TimedStateTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
};

TEST_F(TimedStateTest, TestTimedState){
    //Constructors
    //At (0,0,0), t=0, v=0, a=1
    shared_ptr<Pose2DWithCurvature> pose= make_shared<Pose2DWithCurvature>();
    shared_ptr<TimedState> start_state = make_shared<TimedState>(pose, 0.0, 0.0, 1.0);

    //At (.5, 0, 0), t=1, v=1, a=0
    shared_ptr<Pose2D> pose16= make_shared<Pose2D>(.5, 0.0, 0.0);
    shared_ptr<Pose2DWithCurvature> poseC= make_shared<Pose2DWithCurvature>(pose16, 0.0, 0.0);
    shared_ptr<TimedState> end_state = make_shared<TimedState>(poseC, 1.0, 1.0, 0.0);
    
    EXPECT_NEAR(start_state->acceleration(), 1.0,kEpsilon);
    EXPECT_NEAR(start_state->velocity(), 0.0,kEpsilon);
    EXPECT_NEAR(start_state->t(), 0.0,kEpsilon);
    EXPECT_NEAR(start_state->state()->getTranslation()->x(), 0.0,kEpsilon);

    EXPECT_NEAR(end_state->acceleration(), 0.0,kEpsilon);
    EXPECT_NEAR(end_state->velocity(), 1.0,kEpsilon);
    EXPECT_NEAR(end_state->t(), 1.0,kEpsilon);
    EXPECT_NEAR(end_state->state()->getTranslation()->x(), .5,kEpsilon);

    //Test interpolation
    shared_ptr<TimedState> inter = start_state->interpolate(end_state, .5);
    EXPECT_NEAR(inter->acceleration(), 1.0,kEpsilon);
    EXPECT_NEAR(inter->velocity(), 0.5,kEpsilon);
    EXPECT_NEAR(inter->t(), 0.5,kEpsilon);
    EXPECT_NEAR(inter->state()->getTranslation()->x(), .125,kEpsilon);
}