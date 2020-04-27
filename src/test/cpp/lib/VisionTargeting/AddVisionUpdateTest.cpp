#include "gtest/gtest.h"

#include <opencv2/opencv.hpp>

#include "Subsystems/LimelightManager.h"
#include "Subsystems/Limelight.h"

#include "RobotState.h"

#include "frc/Timer.h"

#include <cmath>
#include <memory>
#include <vector>

class AddVisionUpdateTest : public ::testing::Test {
  protected:
    double kEpsilon = 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}

    std::vector<double> mLargeXY{333.0, 235.0, 608.0, 229.0, 547.0, 341.0, 404.0, 345.0}; //10
    std::vector<double> mStandardXY{333.0, 235.0, 608.0, 229.0, 547.0, 341.0, 404.0, 345.0}; //8
    std::vector<double> mSmallXY{333.0, 235.0, 608.0, 229.0, 547.0, 341.0}; //6

    std::vector<double> mZeroXY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::shared_ptr<Subsystems::Limelight> mLimelight = Subsystems::LimelightManager::getInstance()->getTurretLimelight();
    std::shared_ptr<FRC_7054::RobotState> mRobotState = FRC_7054::RobotState::getInstance();
};

TEST_F(AddVisionUpdateTest, TestXYToTargetCorner)
{
  std::vector<VisionTargeting::TargetCorner> targetCorners;
  mLimelight->XYToTargetCorner(mStandardXY, targetCorners);

  EXPECT_NEAR(333.0, targetCorners.at(0).getY(), kEpsilon);
  EXPECT_NEAR(235.0, targetCorners.at(0).getZ(), kEpsilon);

  EXPECT_NEAR(608.0, targetCorners.at(1).getY(), kEpsilon);
  EXPECT_NEAR(229.0, targetCorners.at(1).getZ(), kEpsilon);

  EXPECT_NEAR(547.0, targetCorners.at(2).getY(), kEpsilon);
  EXPECT_NEAR(341.0, targetCorners.at(2).getZ(), kEpsilon);

  EXPECT_NEAR(404.0, targetCorners.at(3).getY(), kEpsilon);
  EXPECT_NEAR(345.0, targetCorners.at(3).getZ(), kEpsilon);
}  

TEST_F(AddVisionUpdateTest, TestTargetCornerToCVPoint2d)
{
  std::vector<VisionTargeting::TargetCorner> targetCorners;
  mLimelight->XYToTargetCorner(mStandardXY, targetCorners);

  std::vector<cv::Point2d> imagePoints;
  mLimelight->TargetCornerToCVPoint2d(targetCorners, imagePoints);

  EXPECT_NEAR(333.0, imagePoints.at(0).x, kEpsilon);
  EXPECT_NEAR(235.0, imagePoints.at(0).y, kEpsilon);

  EXPECT_NEAR(608.0, imagePoints.at(1).x, kEpsilon);
  EXPECT_NEAR(229.0, imagePoints.at(1).y, kEpsilon);

  EXPECT_NEAR(547.0, imagePoints.at(2).x, kEpsilon);
  EXPECT_NEAR(341.0, imagePoints.at(2).y, kEpsilon);

  EXPECT_NEAR(404.0, imagePoints.at(3).x, kEpsilon);
  EXPECT_NEAR(345.0, imagePoints.at(3).y, kEpsilon);


}

TEST_F(AddVisionUpdateTest, TestFilterTargetCorners)
{
  std::vector<VisionTargeting::TargetCorner> targetCorners;
  mLimelight->XYToTargetCorner(mLargeXY, targetCorners);

  std::vector<VisionTargeting::TargetCorner> filteredTargetCorners;
  mLimelight->filterTargetCorners(targetCorners, filteredTargetCorners);

  EXPECT_NEAR(333.0, filteredTargetCorners.at(0).getY(), kEpsilon);
  EXPECT_NEAR(235.0, filteredTargetCorners.at(0).getZ(), kEpsilon);

  EXPECT_NEAR(608.0, filteredTargetCorners.at(1).getY(), kEpsilon);
  EXPECT_NEAR(229.0, filteredTargetCorners.at(1).getZ(), kEpsilon);

  EXPECT_NEAR(547.0, filteredTargetCorners.at(2).getY(), kEpsilon);
  EXPECT_NEAR(341.0, filteredTargetCorners.at(2).getZ(), kEpsilon);

  EXPECT_NEAR(404.0, filteredTargetCorners.at(3).getY(), kEpsilon);
  EXPECT_NEAR(345.0, filteredTargetCorners.at(3).getZ(), kEpsilon);

  EXPECT_EQ(4, filteredTargetCorners.size());
}

TEST_F(AddVisionUpdateTest, TestGetCornersStandard)
{

  std::vector<cv::Point2d> imagePoints;
  mLimelight->getCorners(mStandardXY, imagePoints);

  EXPECT_NEAR(333.0, imagePoints.at(0).x, kEpsilon);
  EXPECT_NEAR(235.0, imagePoints.at(0).y, kEpsilon);

  EXPECT_NEAR(608.0, imagePoints.at(1).x, kEpsilon);
  EXPECT_NEAR(229.0, imagePoints.at(1).y, kEpsilon);

  EXPECT_NEAR(547.0, imagePoints.at(2).x, kEpsilon);
  EXPECT_NEAR(341.0, imagePoints.at(2).y, kEpsilon);

  EXPECT_NEAR(404.0, imagePoints.at(3).x, kEpsilon);
  EXPECT_NEAR(345.0, imagePoints.at(3).y, kEpsilon);
}

TEST_F(AddVisionUpdateTest, TestGetCornersLarge)
{

  std::vector<cv::Point2d> imagePoints;
  mLimelight->getCorners(mLargeXY, imagePoints);

  EXPECT_NEAR(333.0, imagePoints.at(0).x, kEpsilon);
  EXPECT_NEAR(235.0, imagePoints.at(0).y, kEpsilon);

  EXPECT_NEAR(608.0, imagePoints.at(1).x, kEpsilon);
  EXPECT_NEAR(229.0, imagePoints.at(1).y, kEpsilon);

  EXPECT_NEAR(547.0, imagePoints.at(2).x, kEpsilon);
  EXPECT_NEAR(341.0, imagePoints.at(2).y, kEpsilon);

  EXPECT_NEAR(404.0, imagePoints.at(3).x, kEpsilon);
  EXPECT_NEAR(345.0, imagePoints.at(3).y, kEpsilon);
}

TEST_F(AddVisionUpdateTest, TestGetCornersSmall)
{

  std::vector<cv::Point2d> imagePoints;
  mLimelight->getCorners(mSmallXY, imagePoints);

  
  EXPECT_NEAR(0.0, imagePoints.at(0).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(0).y, kEpsilon);

  EXPECT_NEAR(0.0, imagePoints.at(1).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(1).y, kEpsilon);

  EXPECT_NEAR(0.0, imagePoints.at(2).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(2).y, kEpsilon);

  EXPECT_NEAR(0.0, imagePoints.at(3).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(3).y, kEpsilon);

}

TEST_F(AddVisionUpdateTest, TestGetCornersZero)
{

  std::vector<cv::Point2d> imagePoints;
  mLimelight->getCorners(mZeroXY, imagePoints);

  EXPECT_NEAR(0.0, imagePoints.at(0).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(0).y, kEpsilon);

  EXPECT_NEAR(0.0, imagePoints.at(1).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(1).y, kEpsilon);

  EXPECT_NEAR(0.0, imagePoints.at(2).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(2).y, kEpsilon);

  EXPECT_NEAR(0.0, imagePoints.at(3).x, kEpsilon);
  EXPECT_NEAR(0.0, imagePoints.at(3).y, kEpsilon);
}

TEST_F(AddVisionUpdateTest, TestGetCameraXYZStandard)
{
  //double T = 0;
  //int x = 0;
  //for (int i = 0; i < 100; i++)
  //{
    double timestamp = frc::Timer::GetFPGATimestamp();
    VisionTargeting::TargetInfo info = mLimelight->getCameraXYZ(mStandardXY);
    double dt = frc::Timer::GetFPGATimestamp() - timestamp;
    std::cout<< "dt = " << dt << ", X = " << info.getX() << ", Y = " << info.getY() << ", Z = "<< info.getZ() << ", X theta = "<< info.getXTheta() << ", Y theta = "<< info.getYTheta() << ", Z theta = "<< info.getZTheta() <<  std::endl;  
    //x++;
    //T += dt;
  //}
  
  //std::cout<<"Average Time: " << T/x << std::endl;

}

TEST_F(AddVisionUpdateTest, TestGetCameraXYZLarge)
{
  VisionTargeting::TargetInfo info = mLimelight->getCameraXYZ(mLargeXY);

  std::cout<< "Target Translation: X = " << info.getX() << ", Y = " << info.getY() << ", Z = "<< info.getZ() <<  std::endl;

}

TEST_F(AddVisionUpdateTest, TestGetCameraXYZSmall)
{
  
    
    VisionTargeting::TargetInfo info = mLimelight->getCameraXYZ(mSmallXY);
    
    std::cout<< "Target Translation: X = " << info.getX() << ", Y = " << info.getY() << ", Z = "<< info.getZ() <<  std::endl;  

}

TEST_F(AddVisionUpdateTest, TestGetCameraXYZSZero)
{
  VisionTargeting::TargetInfo info = mLimelight->getCameraXYZ(mZeroXY);

  std::cout<< "Target Translation: X = " << info.getX() << ", Y = " << info.getY() << ", Z = "<< info.getZ() <<  std::endl;

}

TEST_F(AddVisionUpdateTest, TestfindClosestNormal)
{
  
  double normal;
  
  normal = mRobotState->findClosestNormal(0.0);
  EXPECT_NEAR(0.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(85.0);
  EXPECT_NEAR(0.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(-85.0);
  EXPECT_NEAR(0.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(325.0);
  EXPECT_NEAR(0.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(95.0);
  EXPECT_NEAR(180.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(-95.0);
  EXPECT_NEAR(180.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(180.0);
  EXPECT_NEAR(180.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(225.0);
  EXPECT_NEAR(180.0, normal, kEpsilon);
  
  std::shared_ptr<Pose2D> fieldToTarget = std::make_shared<Pose2D>(10.0, 15.0, 22.0);
  mRobotState->updateVisionTargetRotationToNearestNormal(fieldToTarget);

  EXPECT_NEAR(10.0, fieldToTarget->getTranslation()->x(), kEpsilon);

  EXPECT_NEAR(15.0, fieldToTarget->getTranslation()->y(), kEpsilon);

  EXPECT_NEAR(0.0, fieldToTarget->getRotation()->getDegrees(), kEpsilon);

  fieldToTarget = std::make_shared<Pose2D>(10.0, 15.0, 85.0);
  mRobotState->updateVisionTargetRotationToNearestNormal(fieldToTarget);

  EXPECT_NEAR(10.0, fieldToTarget->getTranslation()->x(), kEpsilon);

  EXPECT_NEAR(15.0, fieldToTarget->getTranslation()->y(), kEpsilon);

  EXPECT_NEAR(0.0, fieldToTarget->getRotation()->getDegrees(), kEpsilon);

  fieldToTarget = std::make_shared<Pose2D>(10.0, 15.0, -85.0);
  mRobotState->updateVisionTargetRotationToNearestNormal(fieldToTarget);

  EXPECT_NEAR(10.0, fieldToTarget->getTranslation()->x(), kEpsilon);

  EXPECT_NEAR(15.0, fieldToTarget->getTranslation()->y(), kEpsilon);

  EXPECT_NEAR(0.0, fieldToTarget->getRotation()->getDegrees(), kEpsilon);

  fieldToTarget = std::make_shared<Pose2D>(10.0, 15.0, 95.0);
  mRobotState->updateVisionTargetRotationToNearestNormal(fieldToTarget);

  EXPECT_NEAR(10.0, fieldToTarget->getTranslation()->x(), kEpsilon);

  EXPECT_NEAR(15.0, fieldToTarget->getTranslation()->y(), kEpsilon);

  EXPECT_NEAR(180.0, fieldToTarget->getRotation()->getDegrees(), kEpsilon);

  fieldToTarget = std::make_shared<Pose2D>(10.0, 15.0, -95.0);
  mRobotState->updateVisionTargetRotationToNearestNormal(fieldToTarget);

  EXPECT_NEAR(10.0, fieldToTarget->getTranslation()->x(), kEpsilon);

  EXPECT_NEAR(15.0, fieldToTarget->getTranslation()->y(), kEpsilon);

  EXPECT_NEAR(180.0, fieldToTarget->getRotation()->getDegrees(), kEpsilon);
  
 
  /*
  //uncomment second vector of kTargetNormals and comment the competition vector: these test angle wrap more fully
  double normal;
  
  normal = mRobotState->findClosestNormal(0.0);
  EXPECT_NEAR(270.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(85.0);
  EXPECT_NEAR(180.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(-85.0);
  EXPECT_NEAR(270.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(40.0);
  EXPECT_NEAR(270.0, normal, kEpsilon);
  
  normal = mRobotState->findClosestNormal(50.0);
  EXPECT_NEAR(180.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(230.0);
  EXPECT_NEAR(225.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(245.0);
  EXPECT_NEAR(225.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(250.0);
  EXPECT_NEAR(270.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(205.0);
  EXPECT_NEAR(225.0, normal, kEpsilon);

  normal = mRobotState->findClosestNormal(200.0);
  EXPECT_NEAR(180.0, normal, kEpsilon);
  */
  
}