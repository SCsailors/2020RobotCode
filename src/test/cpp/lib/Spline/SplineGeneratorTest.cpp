#include "gtest/gtest.h"

#include <cmath>
#include <memory>
#include <vector>
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Splines/SplineGenerator.h"
#include "lib/Splines/QuinticHermiteSpline.h"
#include "lib/Util/Util.h"

using namespace std;

class SplineGeneratorTest : public ::testing::Test{
    protected:
     
    double kEpsilon= 1E-3;
    virtual void SetUp(){}
    virtual void TearDown(){}
    shared_ptr<Util> util =make_shared<Util>();
};

TEST_F(SplineGeneratorTest, TestGeneration){
    //shared_ptr<Rotation2D> r1 = make_shared<Rotation2D>();
    //shared_ptr<Rotation2D> r2 = make_shared<Rotation2D>(1.0, -5.0, true);
    //shared_ptr<Translation2D> t1= make_shared<Translation2D>();
    //shared_ptr<Translation2D> t2= make_shared<Translation2D>(15.0, 10.0);
    shared_ptr<Pose2D> p1= make_shared<Pose2D>(make_shared<Translation2D>(), make_shared<Rotation2D>());
    shared_ptr<Pose2D> p2= make_shared<Pose2D>(50.0, 30.0, 45.0);
    shared_ptr<Pose2D> p3= make_shared<Pose2D>(120.0, 90.0, 20.0);
    shared_ptr<Pose2D> p4 = make_shared<Pose2D>(280.0, 90.0, -25.0);
    shared_ptr<QuinticHermiteSpline> Q1=make_shared<QuinticHermiteSpline>(p1, p2);
    shared_ptr<QuinticHermiteSpline> Q2=make_shared<QuinticHermiteSpline>(p2, p3);
    shared_ptr<QuinticHermiteSpline> Q3=make_shared<QuinticHermiteSpline>(p3, p4);
    
    shared_ptr<SplineGenerator> SplineGen= make_shared<SplineGenerator>();
    
    vector<shared_ptr<Pose2DWithCurvature>> samples;
    vector<shared_ptr<QuinticHermiteSpline>> mQuinticHermiteSplines{Q1, Q2, Q3};
    
    //cout<<SplineGen->i<<endl;
    samples=SplineGen->parameterizeSpline(Q1);
    //cout<<"rv size: "<< rv.size()<< endl;
    // test first and last point
    EXPECT_NEAR(0.0, samples.at(0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0.0, samples.at(0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0.0, samples.at(0)->getRotation()->getDegrees(), kEpsilon);
    
    EXPECT_NEAR(50.0, samples.at(samples.size()-1)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(30.0, samples.at(samples.size()-1)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(45.0, samples.at(samples.size()-1)->getRotation()->getDegrees(), kEpsilon);
    


    samples=SplineGen->parameterizeSplines(mQuinticHermiteSplines);
    //for (auto point :samples){
    //cout<< point->getTranslation()->x()<<","<<point->getTranslation()->y()<<","<<point->getRotation()->getDegrees()<<","<<endl;

    //}
    //cout<<"rv size: "<< rv.size()<< endl;
    // test first and last point
    EXPECT_NEAR(0.0, samples.at(0)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0.0, samples.at(0)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0.0, samples.at(0)->getRotation()->getDegrees(), kEpsilon);
    
    EXPECT_NEAR(280.0, samples.at(samples.size()-1)->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(90.0, samples.at(samples.size()-1)->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(-25.0, samples.at(samples.size()-1)->getRotation()->getDegrees(), kEpsilon);
    shared_ptr<Pose2D> p5=make_shared<Pose2D>();
    shared_ptr<Pose2D> p6=make_shared<Pose2D>(make_shared<Translation2D>(15.0, 10.0), make_shared<Rotation2D>(1.0, -5.0, true));
    shared_ptr<QuinticHermiteSpline> mQuintic = make_shared<QuinticHermiteSpline>(p5, p6);

    vector<shared_ptr<Pose2DWithCurvature>> points=SplineGen->parameterizeSpline(mQuintic);
    
    shared_ptr<Twist2D> t=make_shared<Twist2D>();
    double arclength=0.0;
    shared_ptr<Pose2DWithCurvature> cur_pose= points.at(0);
    cout<<cur_pose->getTranslation()->x()<<","<<cur_pose->getTranslation()->y()<<","<<cur_pose->getRotation()->getDegrees()<<","<<cur_pose->getCurvature()<<","<<cur_pose->getDCurvatureDs()<<","<<arclength<<","<<endl;
    
    for (int i=2; i<points.size(); i++ ){
        shared_ptr<Pose2DWithCurvature> sample=points.at(i);
        //bool same=util->epsilonEquals(sample->getTranslation()->x(), cur_pose->getTranslation()->x()) && util->epsilonEquals(sample->getTranslation()->y(), cur_pose->getTranslation()->y());
        t=p1->log(cur_pose->getPose()->inverse()->transformBy(sample->getPose()));
        arclength+=t->dx;
        cur_pose=sample;
        cout<<sample->getTranslation()->x()<<","<<sample->getTranslation()->y()<<","<<sample->getRotation()->getDegrees()<<","<<sample->getCurvature()<<","<<sample->getDCurvatureDs()<<","<<t->dx<<","<<t->dy<<","<<t->dtheta<<","<<arclength<<","<<endl;
    }
    
    EXPECT_NEAR(15.0, cur_pose->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(10.0, cur_pose->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(-78.69006752597981, cur_pose->getRotation()->getDegrees(), kEpsilon);
    EXPECT_NEAR(23.17291953186379, arclength, kEpsilon);

    //EXPECT_NEAR(15.0, samples.at(0)->getTranslation()->x(), kEpsilon);
    //EXPECT_NEAR(10.0, samples.at(0)->getTranslation()->y(), kEpsilon);
    //EXPECT_NEAR(0.0, samples.at(0)->getRotation()->getDegrees(), kEpsilon);
    
    //EXPECT_NEAR(15.0, samples.at(1)->getTranslation()->x(), kEpsilon);
    //EXPECT_NEAR(10.0, samples.at(1)->getTranslation()->y(), kEpsilon);
    //EXPECT_NEAR(0.0, samples.at(1)->getRotation()->getDegrees(), kEpsilon);

}
