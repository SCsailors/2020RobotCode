#include "gtest/gtest.h"

#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Trajectory/TrajectoryIterator.h"

#include <cmath>
#include <memory>
#include <vector>
using namespace std;

class TrajectoryIteratorTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
};

TEST_F(TrajectoryIteratorTest, TestIteration){
    
}