#include "gtest/gtest.h"

#include <memory>
#include <cmath>
using namespace std;

#include "lib/Splines/QuinticSpline.h"
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"

class QuinticSplineTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
};

TEST_F(QuinticSplineTest, splineGeneration){
    //Test constructors
    shared_ptr<QuinticSpline> s = make_shared<QuinticSpline>();
    EXPECT_NEAR(0, s->getVelocity(0.0), kEpsilon);
    EXPECT_NEAR(0, s->getCurvature(0.0), kEpsilon);
    EXPECT_NEAR(0, s->getDCurvature(0.0), kEpsilon);
    EXPECT_NEAR(0, s->getHeading(0.0)->getDegrees(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(0.0)->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(0.0)->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getRotation()->getDegrees(), kEpsilon);

    EXPECT_NEAR(0, s->getVelocity(1.0), kEpsilon);
    EXPECT_NEAR(0, s->getCurvature(1.0), kEpsilon);
    EXPECT_NEAR(0, s->getDCurvature(1.0), kEpsilon);
    EXPECT_NEAR(0, s->getHeading(1.0)->getDegrees(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(1.0)->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(1.0)->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(1.0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(1.0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(1.0)->getRotation()->getDegrees(), kEpsilon);

    shared_ptr<Pose2D> pose1= make_shared<Pose2D>();
    shared_ptr<Pose2D> pose2= make_shared<Pose2D>(3.0, 4.0, 90.0);
    s=make_shared<QuinticSpline>(pose1, pose2, true);
    //EXPECT_NEAR(0, s->getVelocity(0.0), kEpsilon);
    //EXPECT_NEAR(0, s->getCurvature(0.0), kEpsilon);
    //EXPECT_NEAR(0, s->getDCurvature(0.0), kEpsilon);
    EXPECT_NEAR(0, s->getHeading(0.0)->getDegrees(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(0.0)->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(0.0)->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getRotation()->getDegrees(), kEpsilon);

    //EXPECT_NEAR(0, s->getVelocity(1.0), kEpsilon);
    //EXPECT_NEAR(0, s->getCurvature(1.0), kEpsilon);
    //EXPECT_NEAR(0, s->getDCurvature(1.0), kEpsilon);
    EXPECT_NEAR(90, s->getHeading(1.0)->getDegrees(), kEpsilon);
    EXPECT_NEAR(3, s->getPoint(1.0)->x(), kEpsilon);
    EXPECT_NEAR(4, s->getPoint(1.0)->y(), kEpsilon);
    EXPECT_NEAR(3, s->getPose2D(1.0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(4, s->getPose2D(1.0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(90, s->getPose2D(1.0)->getRotation()->getDegrees(), kEpsilon);

    EXPECT_NEAR(90, s->getPose2DWithCurvature(1.0)->getRotation()->getDegrees(), kEpsilon);
}