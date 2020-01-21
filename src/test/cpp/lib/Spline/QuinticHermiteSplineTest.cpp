#include "gtest/gtest.h"

#include <memory>
#include <cmath>
using namespace std;

#include "lib/Splines/QuinticHermiteSpline.h"
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"

class QuinticHermiteSplineTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
    shared_ptr<QuinticHermiteSpline> s = make_shared<QuinticHermiteSpline>(make_shared<Pose2D>(), make_shared<Pose2D>(3.0, 4.0, 90.0));
    shared_ptr<QuinticHermiteSpline> s1 = make_shared<QuinticHermiteSpline>(make_shared<Pose2D>(3.0, 4.0, 90.0), make_shared<Pose2D>(10.0, 15.0, 90.0));
    shared_ptr<QuinticHermiteSpline> s2 = make_shared<QuinticHermiteSpline>(make_shared<Pose2D>(10.0, 15.0, 90.0), make_shared<Pose2D>(20.0, 15.0, 90.0));

};

TEST_F(QuinticHermiteSplineTest, splineGenerationTest){
    //EXPECT_NEAR(0, s->getVelocity(0.0), kEpsilon);
    //EXPECT_NEAR(0, s->getCurvature(0.0), kEpsilon);
    //EXPECT_NEAR(0, s->getDCurvature(0.0), kEpsilon);
    EXPECT_NEAR(0, s->getHeading(0.0)->getDegrees(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(0.0)->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPoint(0.0)->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, s->getPose2D(0.0)->getRotation()->getDegrees(), kEpsilon);

    EXPECT_NEAR(90, s->getHeading(1.0)->getDegrees(), kEpsilon);
    EXPECT_NEAR(3, s->getPoint(1.0)->x(), kEpsilon);
    EXPECT_NEAR(4, s->getPoint(1.0)->y(), kEpsilon);
    EXPECT_NEAR(3, s->getPose2D(1.0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(4, s->getPose2D(1.0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(90, s->getPose2D(1.0)->getRotation()->getDegrees(), kEpsilon);

    EXPECT_NEAR(90, s->getPose2DWithCurvature(1.0)->getRotation()->getDegrees(), kEpsilon);
    vector<shared_ptr<QuinticHermiteSpline>> splines{s, s1, s2};
    /*
    //cout<<"X, Y, Theta"<< endl;
    for (double i=0.0; i<=1.0; i+=.05){
        shared_ptr<Pose2D> p1= s->getPose2D(i);
        //cout<<i<<": "<<p1->getTranslation()->x()<<","<<p1->getTranslation()->y()<<","<<p1->getRotation()->getDegrees()<<","<<endl;
    }
    for (double i=0.0; i<=1.0; i+=.05){
        shared_ptr<Pose2D> p1= s1->getPose2D(i);
        //cout<<i<<": "<<p1->getTranslation()->x()<<","<<p1->getTranslation()->y()<<","<<p1->getRotation()->getDegrees()<<","<<endl;
    }

    for (double i=0.0; i<=1.0; i+=.05){
        shared_ptr<Pose2D> p1= s2->getPose2D(i);
        //cout<<i<<": "<<p1->getTranslation()->x()<<","<<p1->getTranslation()->y()<<","<<p1->getRotation()->getDegrees()<<","<<endl;
    }

    s->runOptimizationIteration(splines);
    //cout<<"After Optimization"<<endl;
    for (double i=0.0; i<=1.0; i+=.05){
        shared_ptr<Pose2D> p1= s->getPose2D(i);
      //  cout<<i<<": "<<p1->getTranslation()->x()<<","<<p1->getTranslation()->y()<<","<<p1->getRotation()->getDegrees()<<","<<endl;
    }
    for (double i=0.0; i<=1.0; i+=.05){
        shared_ptr<Pose2D> p1= s1->getPose2D(i);
        //cout<<i<<": "<<p1->getTranslation()->x()<<","<<p1->getTranslation()->y()<<","<<p1->getRotation()->getDegrees()<<","<<endl;
    }

    for (double i=0.0; i<=1.0; i+=.05){
        shared_ptr<Pose2D> p1= s2->getPose2D(i);
        //cout<<i<<": "<<p1->getTranslation()->x()<<","<<p1->getTranslation()->y()<<","<<p1->getRotation()->getDegrees()<<","<<endl;
    }
    */
}