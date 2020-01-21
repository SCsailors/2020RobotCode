#include "gtest/gtest.h"

#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Twist2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Trajectory/Timing/TimingUtil.h"
#include "lib/Trajectory/Timing/TimingConstraint.h"



#include <cmath>
#include <memory>
using namespace std;

class TimingUtilTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
    //shared_ptr<
    //shared_ptr<TimingUtil> TimUtil = make_shared<TimingUtil>();
    
};

TEST_F(TimingUtilTest, TestTimingUtil){

}