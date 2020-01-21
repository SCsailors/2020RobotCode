#include "gtest/gtest.h"

#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Rotation2D.h"
#include "lib/Geometry/Twist2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"

#include <cmath>
#include <memory>
using namespace std;

class SE2MathTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
};

TEST_F(SE2MathTest, TestRotation2D){
    //test constructors
    shared_ptr<Rotation2D> rot1 = make_shared<Rotation2D>();
    EXPECT_NEAR(1, rot1->cos(), kEpsilon);
    EXPECT_NEAR(0, rot1->sin(), kEpsilon);
    EXPECT_NEAR(0, rot1->getDegrees(), kEpsilon);
    EXPECT_NEAR(0, rot1->getRadians(), kEpsilon);

    rot1=make_shared<Rotation2D>(1.0,1.0, true);
    EXPECT_NEAR(sqrt(2.0)/2.0, rot1->cos(), kEpsilon);
    EXPECT_NEAR(sqrt(2.0)/2.0, rot1->sin(), kEpsilon);
    EXPECT_NEAR(45, rot1->getDegrees(), kEpsilon);
    EXPECT_NEAR(pi/4.0, rot1->getRadians(), kEpsilon);

    rot1=rot1->fromRadians(pi/2.0);
    EXPECT_NEAR(0, rot1->cos(), kEpsilon);
    EXPECT_NEAR(1, rot1->sin(), kEpsilon);
    EXPECT_NEAR(90, rot1->getDegrees(), kEpsilon);
    EXPECT_NEAR(pi/2.0, rot1->getRadians(), kEpsilon);

    rot1=rot1->fromDegrees(270.0);
    EXPECT_NEAR(0, rot1->cos(), kEpsilon);
    EXPECT_NEAR(-1, rot1->sin(), kEpsilon);
    EXPECT_NEAR(-90, rot1->getDegrees(), kEpsilon);
    EXPECT_NEAR(-pi/2.0, rot1->getRadians(), kEpsilon);

    //test inversion
    rot1=rot1->fromDegrees(270.0);
    shared_ptr<Rotation2D> rot2=rot1->inverse();
    EXPECT_NEAR(0, rot2->cos(), kEpsilon);
    EXPECT_NEAR(1, rot2->sin(), kEpsilon);
    EXPECT_NEAR(90, rot2->getDegrees(), kEpsilon);
    EXPECT_NEAR(pi/2.0, rot2->getRadians(), kEpsilon);

    rot1=rot1->fromDegrees(1.0);
    rot2=rot1->inverse();
    EXPECT_NEAR(rot1->cos(), rot2->cos(), kEpsilon);
    EXPECT_NEAR(-rot1->sin(), rot2->sin(), kEpsilon);
    EXPECT_NEAR(-1, rot2->getDegrees(), kEpsilon);

    //Test RotateBy
    rot1=rot1->fromDegrees(45.0);
    rot2=rot1->fromDegrees(45.0);
    shared_ptr<Rotation2D> rot3= rot1->rotateBy(rot2);
    EXPECT_NEAR(0, rot3->cos(), kEpsilon);
    EXPECT_NEAR(1, rot3->sin(), kEpsilon);
    EXPECT_NEAR(90, rot3->getDegrees(), kEpsilon);
    EXPECT_NEAR(pi/2.0, rot3->getRadians(), kEpsilon);
    
    //rotation of inverse should be identity
    rot3=make_shared<Rotation2D>();
    rot1=rot1->fromDegrees(21.45);
    rot2=rot1->rotateBy(rot1->inverse());
    EXPECT_NEAR(rot3->cos(), rot2->cos(), kEpsilon);
    EXPECT_NEAR(rot3->sin(), rot2->sin(), kEpsilon);
    EXPECT_NEAR(rot3->getDegrees(), rot2->getDegrees(), kEpsilon);

    //Test interpolation
    rot1=rot1->fromDegrees(45.0);
    rot2=rot1->fromDegrees(135.0);
    rot3=rot1->interpolate(rot2, .5);
    EXPECT_NEAR(90, rot3->getDegrees(), kEpsilon);

    rot1=rot1->fromDegrees(45.0);
    rot2=rot1->fromDegrees(135.0);
    rot3=rot1->interpolate(rot2, .75);
    EXPECT_NEAR(112.5, rot3->getDegrees(), kEpsilon);    

    rot1=rot1->fromDegrees(45.0);
    rot2=rot1->fromDegrees(-45.0);
    rot3=rot1->interpolate(rot2, .5);
    EXPECT_NEAR(0, rot3->getDegrees(), kEpsilon);

    rot1=rot1->fromDegrees(45.0);
    rot2=rot1->fromDegrees(45.0);
    rot3=rot1->interpolate(rot2, .5);
    EXPECT_NEAR(45, rot3->getDegrees(), kEpsilon);

}

TEST_F(SE2MathTest, TestTranslation2D){
    //test constructors
    shared_ptr<Translation2D> pos1 = make_shared<Translation2D>();
    EXPECT_NEAR(0, pos1->x(), kEpsilon);
    EXPECT_NEAR(0, pos1->y(), kEpsilon);
    EXPECT_NEAR(0, pos1->norm(), kEpsilon);

    pos1= make_shared<Translation2D>(3.0, 4.0);
    EXPECT_NEAR(3, pos1->x(), kEpsilon);
    EXPECT_NEAR(4, pos1->y(), kEpsilon);
    EXPECT_NEAR(5, pos1->norm(), kEpsilon);
    
    //test inversion
    pos1=make_shared<Translation2D>(3.152, 4.1666);
    shared_ptr<Translation2D> pos2=pos1->inverse();
    EXPECT_NEAR(-pos1->x(), pos2->x(), kEpsilon);
    EXPECT_NEAR(-pos1->y(), pos2->y(), kEpsilon);
    EXPECT_NEAR(pos1->norm(), pos2->norm(), kEpsilon);

    //Test rotateBy
    pos1=make_shared<Translation2D>(2.0, 0.0);
    shared_ptr<Rotation2D> rot1= make_shared<Rotation2D>(0.0, 1.0, false);
    rot1=rot1->fromDegrees(90.0);
    pos2=pos1->rotateBy(rot1);
    EXPECT_NEAR(0, pos2->x(), kEpsilon);
    EXPECT_NEAR(2, pos2->y(), kEpsilon);
    EXPECT_NEAR(pos1->norm(), pos2->norm(), kEpsilon);

    pos1=make_shared<Translation2D>(2.0, 0.0);
    rot1=rot1->fromDegrees(-45.0);
    pos2=pos1->rotateBy(rot1);
    EXPECT_NEAR(sqrt(2.0), pos2->x(), kEpsilon);
    EXPECT_NEAR(-sqrt(2.0), pos2->y(), kEpsilon);
    EXPECT_NEAR(pos1->norm(), pos2->norm(), kEpsilon);

    //test translateBy
    pos1=make_shared<Translation2D>(2.0, 1.0);
    pos2=make_shared<Translation2D>(-2.0, 0.0);
    shared_ptr<Translation2D> pos3=pos1->translateBy(pos2);
    EXPECT_NEAR(0, pos3->x(), kEpsilon);
    EXPECT_NEAR(1, pos3->y(), kEpsilon);
    EXPECT_NEAR(1, pos3->norm(), kEpsilon);

    //test inverse
    shared_ptr<Translation2D> identity=make_shared<Translation2D>();
    pos1=make_shared<Translation2D>(2.16612, -23.55);
    pos2=pos1->translateBy(pos1->inverse());
    EXPECT_NEAR(identity->y(), pos2->x(), kEpsilon);
    EXPECT_NEAR(identity->x(), pos2->y(), kEpsilon);
    EXPECT_NEAR(identity->norm(), pos2->norm(), kEpsilon);

    //test interpolation
    pos1=make_shared<Translation2D>(0.0, 1.0);
    pos2=make_shared<Translation2D>(10.0, -1.0);
    pos3=pos1->interpolate(pos2, .75);
    EXPECT_NEAR(7.5, pos3->x(), kEpsilon);
    EXPECT_NEAR(-.5, pos3->y(), kEpsilon);


}

TEST_F(SE2MathTest, TestPose2D){
    //test constructors
    shared_ptr<Pose2D> pose1=make_shared<Pose2D>();
    EXPECT_NEAR(0, pose1->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, pose1->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, pose1->getRotation()->getDegrees(), kEpsilon);

    pose1=make_shared<Pose2D>(3.0, 4.0, 45.0);
    EXPECT_NEAR(3, pose1->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(4, pose1->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(45, pose1->getRotation()->getDegrees(), kEpsilon);

    shared_ptr<Rotation2D> rot1=make_shared<Rotation2D>();
    shared_ptr<Translation2D> trans=make_shared<Translation2D>(3.0, 4.0);
    pose1=make_shared<Pose2D>(trans, rot1->fromDegrees(45.0));
    EXPECT_NEAR(3, pose1->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(4, pose1->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(45, pose1->getRotation()->getDegrees(), kEpsilon);

    //Test transformation
    pose1=make_shared<Pose2D>(3.0, 4.0, 90.0);
    shared_ptr<Pose2D> pose2=make_shared<Pose2D>(1.0, 0.0, 0.0);
    shared_ptr<Pose2D> pose3= pose1->transformBy(pose2);
    EXPECT_NEAR(3, pose3->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(5, pose3->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(90, pose3->getRotation()->getDegrees(), kEpsilon);
    
    pose1=make_shared<Pose2D>(3.0, 4.0, 90.0);
    pose2=make_shared<Pose2D>(1.0, 0.0, -90.0);
    pose3= pose1->transformBy(pose2);
    EXPECT_NEAR(3, pose3->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(5, pose3->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, pose3->getRotation()->getDegrees(), kEpsilon);

    //Pose translated by inverse is its identity
    shared_ptr<Pose2D> identity= make_shared<Pose2D>();
    pose1=make_shared<Pose2D>(3.12123424, 8.286395, 93.1235);
    pose2=pose1->transformBy(pose1->inverse());
    EXPECT_NEAR(identity->getTranslation()->x(), pose2->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(identity->getTranslation()->y(), pose2->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(identity->getRotation()->getDegrees(), pose2->getRotation()->getDegrees(), kEpsilon);

    //Test interpolation 
    //    Movement from pose1 to pose2 along a circle with radius of 10 units centered at (3, -6)
    pose1= make_shared<Pose2D>(3.0, 4.0, 90.0);
    pose2=make_shared<Pose2D>(13.0, -6.0, 0.0);
    pose3=pose1->interpolate(pose2, .5);
    double expected_angle_radians= pi/4.0;
    EXPECT_NEAR(3.0+10.0*cos(expected_angle_radians), pose3->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(-6.0+10.0*sin(expected_angle_radians), pose3->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(expected_angle_radians, pose3->getRotation()->getRadians(), kEpsilon);

    pose1= make_shared<Pose2D>(3.0, 4.0, 90.0);
    pose2=make_shared<Pose2D>(13.0, -6.0, 0.0);
    pose3=pose1->interpolate(pose2, .75);
    expected_angle_radians= pi/8.0;
    EXPECT_NEAR(3.0+10.0*cos(expected_angle_radians), pose3->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(-6.0+10.0*sin(expected_angle_radians), pose3->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(expected_angle_radians, pose3->getRotation()->getRadians(), kEpsilon);

}

TEST_F(SE2MathTest, TestPose2DWithCurvature){
    shared_ptr<Pose2DWithCurvature> pos1= make_shared<Pose2DWithCurvature>();
    EXPECT_NEAR(0, pos1->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, pos1->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, pos1->getRotation()->getDegrees(), kEpsilon);
    EXPECT_NEAR(0, pos1->getCurvature(), kEpsilon);
    EXPECT_NEAR(0, pos1->getDCurvatureDs(), kEpsilon);

    shared_ptr<Pose2D> pose1=make_shared<Pose2D>(1.0, 3.0, 45.0);
    pos1=make_shared<Pose2DWithCurvature>(pose1, .5, .1);
    EXPECT_NEAR(1, pos1->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(3, pos1->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(45, pos1->getRotation()->getDegrees(), kEpsilon);
    EXPECT_NEAR(.5, pos1->getCurvature(), kEpsilon);
    EXPECT_NEAR(.1, pos1->getDCurvatureDs(), kEpsilon);


    //transform
    pose1=make_shared<Pose2D>(3.0, 4.0, 90.0);
    shared_ptr<Pose2D> pose2=make_shared<Pose2D>(1.0, 0.0, 0.0);
    pos1=make_shared<Pose2DWithCurvature>(pose1, .5, .1);
    shared_ptr<Pose2DWithCurvature> pose3= pos1->transformBy(pose2);
    EXPECT_NEAR(3, pose3->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(5, pose3->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(90, pose3->getRotation()->getDegrees(), kEpsilon);
    EXPECT_NEAR(.5, pose3->getCurvature(), kEpsilon);
    EXPECT_NEAR(.1, pose3->getDCurvatureDs(), kEpsilon);    

    //interpolation
    pose1= make_shared<Pose2D>(3.0, 4.0, 90.0);
    pose2=make_shared<Pose2D>(13.0, -6.0, 0.0);
    pos1=make_shared<Pose2DWithCurvature>(pose1, .5, .1);
    shared_ptr<Pose2DWithCurvature> pos2= make_shared<Pose2DWithCurvature>(pose2, 1.0, .2);
    
    pose3=pos1->interpolate(pos2, .5);
    double expected_angle_radians= pi/4.0;
    EXPECT_NEAR(3.0+10.0*cos(expected_angle_radians), pose3->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(-6.0+10.0*sin(expected_angle_radians), pose3->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(expected_angle_radians, pose3->getRotation()->getRadians(), kEpsilon);
    EXPECT_NEAR(.75, pose3->getCurvature(), kEpsilon);
    EXPECT_NEAR(.15, pose3->getDCurvatureDs(), kEpsilon); 

    //Distance
    pose1=make_shared<Pose2D>(3.0, 4.0, 0.0);
    pos1=make_shared<Pose2DWithCurvature>(pose1, .5, .1);
    pose3=make_shared<Pose2DWithCurvature>();
    EXPECT_NEAR(5, pose3->getDistance(pos1), kEpsilon);
}

TEST_F(SE2MathTest, TestTwist2D){
    //test constructors and exponentiation (integrate twist to obtain a Pose2D)
    shared_ptr<Twist2D> twist= make_shared<Twist2D>(1.0, 0.0, 0.0);
    shared_ptr<Pose2D> pose= make_shared<Pose2D>();
    shared_ptr<Pose2D> pose2=pose->exp(twist);
    
    EXPECT_NEAR(1, pose2->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, pose2->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, pose2->getRotation()->getDegrees(), kEpsilon);
    

    //Scaled
    twist=make_shared<Twist2D>(1.0, 0.0, 0.0);
    pose=pose->exp(twist->scaled(2.5));
    EXPECT_NEAR(2.5, pose->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(0, pose->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(0, pose->getRotation()->getDegrees(), kEpsilon);

    //Logarithm (find the twist to apply to obtain a given Pose2D)
    pose=make_shared<Pose2D>(2.0, 2.0, 90.0);
    twist=pose->log(pose);
    EXPECT_NEAR(pi, twist->dx, kEpsilon);
    EXPECT_NEAR(0, twist->dy, kEpsilon);
    EXPECT_NEAR(pi/2.0, twist->dtheta, kEpsilon);

    //Logarithm is the inverse of exponentiation
    shared_ptr<Pose2D> new_pose=pose->exp(twist);
    EXPECT_NEAR(new_pose->getTranslation()->x(), pose->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(new_pose->getTranslation()->y(), pose->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(new_pose->getRotation()->getDegrees(), pose->getRotation()->getDegrees(), kEpsilon);

}