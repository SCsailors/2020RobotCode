#include "gtest/gtest.h"

#include "lib/Trajectory/TrajectoryUtil.h"

#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"

#include <cmath>
#include <vector>
#include <memory>
using namespace std;

class TrajectoryUtilTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
    
};

TEST_F(TrajectoryUtilTest, TestUtil){
    shared_ptr<Pose2D> p1= make_shared<Pose2D>(make_shared<Translation2D>(), make_shared<Rotation2D>());
    shared_ptr<Pose2D> p2= make_shared<Pose2D>(50.0, 30.0, 45.0);
    shared_ptr<Pose2D> p3= make_shared<Pose2D>(120.0, 90.0, 20.0);
    shared_ptr<Pose2D> p4 = make_shared<Pose2D>(280.0, 90.0, -25.0);
    
    
    shared_ptr<TrajectoryUtil> trajectoryUtil = make_shared<TrajectoryUtil>();
    vector<shared_ptr<Pose2D>> waypoints{p1,p2,p3,p4};
    vector<shared_ptr<Pose2DWithCurvature>> generatedPoints;

    
    cout<<waypoints.size()<<endl;
    generatedPoints=trajectoryUtil->trajectoryFromSplineWaypoints(waypoints);
    EXPECT_EQ(3, trajectoryUtil->mQuintic.size());

    //for (auto point :generatedPoints){
    //cout<< point->getTranslation()->x()<<","<<point->getTranslation()->y()<<","<<point->getRotation()->getDegrees()<<","<<endl;

    //}
}