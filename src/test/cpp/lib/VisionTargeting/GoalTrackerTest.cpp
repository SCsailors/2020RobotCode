#include "gtest/gtest.h"

#include "lib/Vision/GoalTracker.h"
#include "lib/Vision/GoalTrack.h"

#include "lib/Geometry/Pose2D.h"
#include "lib/Vision/TimedPose2D.h"

#include <cmath>
#include <memory>

class GoalTrackerTest : public ::testing::Test {
  protected:
    double kEpsilon = 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}

    VisionTargeting::GoalTracker tracker{};
};

TEST_F(GoalTrackerTest, TestUpdate)
{
  
  std::shared_ptr<Pose2D> pose1 = std::make_shared<Pose2D>(10.0, 15.0, 30.0); //Track 1
  std::shared_ptr<Pose2D> pose2 = std::make_shared<Pose2D>(15.0, 30.0, 20.0); //Track 2
  std::shared_ptr<Pose2D> pose3 = std::make_shared<Pose2D>(15.0, 10.0, 20.0); //Track 1
  std::shared_ptr<Pose2D> pose4 = std::make_shared<Pose2D>(15.0, 20.0, 50.0); //Track 3
  std::shared_ptr<Pose2D> pose5 = std::make_shared<Pose2D>(20.0, 25.0, 20.0); // Track 2
  std::shared_ptr<Pose2D> pose6 = std::make_shared<Pose2D>(10.0, 25.0, 55.0); //Track 3

  std::shared_ptr<TimedPose2D> tp1 = std::make_shared<TimedPose2D>(0.01, pose1);
  std::shared_ptr<TimedPose2D> tp2 = std::make_shared<TimedPose2D>(0.02, pose2);
  std::shared_ptr<TimedPose2D> tp3 = std::make_shared<TimedPose2D>(0.03, pose3);
  std::shared_ptr<TimedPose2D> tp4 = std::make_shared<TimedPose2D>(0.04, pose4);
  std::shared_ptr<TimedPose2D> tp5 = std::make_shared<TimedPose2D>(0.05, pose5);
  std::shared_ptr<TimedPose2D> tp6 = std::make_shared<TimedPose2D>(0.06, pose6);

  std::vector<std::shared_ptr<TimedPose2D>> fieldToGoals{tp1, tp2, tp3, tp4, tp5, tp6};
  //std::vector<std::shared_ptr<TimedPose2D>> fieldToGoals{tp1, tp3};

  tracker.update(fieldToGoals);

  EXPECT_EQ(3, tracker.mCurrentTracks.size());

  EXPECT_EQ(2, tracker.mCurrentTracks.at(0)->mObservedPositions.pastObjects.size());
  EXPECT_FALSE(tracker.mCurrentTracks.at(0)->mObservedPositions.isEmpty());
  EXPECT_TRUE(tracker.mCurrentTracks.at(0)->isAlive());
  EXPECT_EQ(tracker.mCurrentTracks.at(0)->getId(), 0);
  EXPECT_EQ(2, tracker.mCurrentTracks.at(1)->mObservedPositions.pastObjects.size());
  EXPECT_EQ(2, tracker.mCurrentTracks.at(2)->mObservedPositions.pastObjects.size());
  

  //interpolation is just an approximation, should really just average the two, so larger epsilons
  EXPECT_NEAR(12.5, tracker.mCurrentTracks.at(0)->getSmoothedPosition()->getTranslation()->x(), kEpsilon);
  EXPECT_NEAR(12.5, tracker.mCurrentTracks.at(0)->getSmoothedPosition()->getTranslation()->y(), kEpsilon);
  EXPECT_NEAR(25, tracker.mCurrentTracks.at(0)->getSmoothedPosition()->getRotation()->getDegrees(), kEpsilon);

  EXPECT_NEAR(17.5, tracker.mCurrentTracks.at(1)->getSmoothedPosition()->getTranslation()->x(), kEpsilon);
  EXPECT_NEAR(27.5, tracker.mCurrentTracks.at(1)->getSmoothedPosition()->getTranslation()->y(), kEpsilon);
  EXPECT_NEAR(20.0, tracker.mCurrentTracks.at(1)->getSmoothedPosition()->getRotation()->getDegrees(), kEpsilon);

  EXPECT_NEAR(12.5, tracker.mCurrentTracks.at(2)->getSmoothedPosition()->getTranslation()->x(), kEpsilon);
  EXPECT_NEAR(22.5, tracker.mCurrentTracks.at(2)->getSmoothedPosition()->getTranslation()->y(), kEpsilon);
  EXPECT_NEAR(52.5, tracker.mCurrentTracks.at(2)->getSmoothedPosition()->getRotation()->getDegrees(), kEpsilon);
  
}

TEST_F(GoalTrackerTest, TestTreeMap)
{
  TreeMap<double, std::shared_ptr<Pose2D>> mObservedPositions{100};

  std::shared_ptr<Pose2D> pose1 = std::make_shared<Pose2D>(10.0, 15.0, 30.0); //Track 1
  std::shared_ptr<Pose2D> pose2 = std::make_shared<Pose2D>(15.0, 30.0, 20.0); //Track 2
  std::shared_ptr<Pose2D> pose3 = std::make_shared<Pose2D>(15.0, 10.0, 20.0); //Track 1
  std::shared_ptr<Pose2D> pose4 = std::make_shared<Pose2D>(15.0, 20.0, 50.0); //Track 3
  std::shared_ptr<Pose2D> pose5 = std::make_shared<Pose2D>(20.0, 25.0, 20.0); // Track 2
  std::shared_ptr<Pose2D> pose6 = std::make_shared<Pose2D>(10.0, 25.0, 55.0); //Track 3
/*
  mObservedPositions.put(.01, pose1);
  EXPECT_EQ(mObservedPositions.pastObjects.size(), 1);

  mObservedPositions.put(.01, pose2);
  EXPECT_EQ(mObservedPositions.pastObjects.size(), 2);
*/
}