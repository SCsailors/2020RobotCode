#include "gtest/gtest.h"

#include "Robot.h"
#include "lib/Geometry/Pose2D.h"
#include "lib/Geometry/Pose2DWithCurvature.h"
#include "lib/Geometry/Twist2D.h"

#include "Planners/DriveMotionPlanner.h"
#include "lib/Trajectory/Timing/TimedState.h"
#include "lib/Trajectory/TimedView.h"
#include "lib/Trajectory/TrajectoryIterator.h"


#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
using namespace std;

class DriveMotionPlannerTest : public ::testing::Test{
    protected:
    double pi= 3.14159265358979238463;
    double kEpsilon= 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
    shared_ptr<DriveMotionPlanner> motion_planner= make_shared<DriveMotionPlanner>();
    shared_ptr<DriveMotionPlanner> motion_planner1=make_shared<DriveMotionPlanner>();
    shared_ptr<DriveMotionPlanner> motion_planner2=make_shared<DriveMotionPlanner>();
     shared_ptr<Pose2D> p1=make_shared<Pose2D>(0.0, 0.0, 0.0);
    shared_ptr<Pose2D> p2=make_shared<Pose2D>(120.0, -36.0, 0.0);
    shared_ptr<Pose2D> p3=make_shared<Pose2D>(240.0, -36.0, 0.0);
    
    vector<shared_ptr<Pose2D>> waypoints{p1, p2, p3};
    
    vector<shared_ptr<TimedState>> points= motion_planner->generateTrajectory(false, waypoints, 120.0, 120.0, 10.0);
    vector<shared_ptr<TimedState>> points1= motion_planner1->generateTrajectory(false, waypoints, 120.0, 120.0, 10.0);
    vector<shared_ptr<TimedState>> points2= motion_planner2->generateTrajectory(false, waypoints, 120.0, 120.0, 10.0);
    
      

    shared_ptr<TimedView> timed_view= make_shared<TimedView>(points);
    shared_ptr<TimedView> timed_view1= make_shared<TimedView>(points1);
    shared_ptr<TimedView> timed_view2= make_shared<TimedView>(points2);

    shared_ptr<TrajectoryIterator> trajectory_iterator=make_shared<TrajectoryIterator>(timed_view);
    shared_ptr<TrajectoryIterator> trajectory_iterator1=make_shared<TrajectoryIterator>(timed_view1);
    shared_ptr<TrajectoryIterator> trajectory_iterator2=make_shared<TrajectoryIterator>(timed_view2);
};

TEST_F(DriveMotionPlannerTest, TestPIDTuner){
    double velocity=4500.0;
    shared_ptr<DriveMotionPlanner::Output> output = motion_planner->updatePIDTuner(velocity);
    
    cout<<output->left_velocity<<","<<output->right_velocity<<","<<output->left_acceleration<<","<<output->right_acceleration<<","<<output->left_feedforward_voltage<<","<<output->right_feedforward_voltage<<","<<endl;
    //EXPECT_NEAR(velocity, output->left_velocity, kEpsilon);
    //EXPECT_NEAR(velocity, output->right_velocity, kEpsilon);
}
//create better tests once generics are added
TEST_F(DriveMotionPlannerTest, TestSwerveRight){
   
   //print x, y, theta, velocity, acceleration, time of points
   for (auto point: points){
    //    cout<<point->state()->getTranslation()->x()<<","<<point->state()->getTranslation()->y()<<","<<point->state()->getRotation()->getDegrees()<<","<<point->velocity()<<","<<point->acceleration()<<","<<point->t()<<","<<endl;
    } 
     
    motion_planner->setTrajectory(trajectory_iterator);
    motion_planner1->setTrajectory(trajectory_iterator1);
    motion_planner2->setTrajectory(trajectory_iterator2);

    
    double t=0.0;
    shared_ptr<DriveMotionPlanner::Output> output= make_shared<DriveMotionPlanner::Output>();
    shared_ptr<DriveMotionPlanner::Output> output1= make_shared<DriveMotionPlanner::Output>();
    shared_ptr<DriveMotionPlanner::Output> output2= make_shared<DriveMotionPlanner::Output>();
    shared_ptr<Pose2D> pose=make_shared<Pose2D>();
    shared_ptr<Pose2D> pose1=make_shared<Pose2D>();
    shared_ptr<Pose2D> pose2=make_shared<Pose2D>();
    int i=0;
    motion_planner->mFollowerType=DriveMotionPlanner::FEEDFORWARD_ONLY;
    motion_planner1->mFollowerType=DriveMotionPlanner::PID;
    motion_planner2->mFollowerType=DriveMotionPlanner::NONLINEAR_FEEDBACK;
    //motion_planner->setFollowerType(DriveMotionPlanner::FollowerType::FEEDFORWARD_ONLY);
    //motion_planner1->setFollowerType(DriveMotionPlanner::FollowerType::PID);
    //motion_planner2->setFollowerType(DriveMotionPlanner::FollowerType::NONLINEAR_FEEDBACK);

    while(!motion_planner->isDone()){
        
        pose=motion_planner->mSetpoint->state()->getPose();
        pose1=motion_planner1->mSetpoint->state()->getPose();
        pose2=motion_planner2->mSetpoint->state()->getPose();
        //pose->transformBy(make_shared<Pose2D>(-5.0, 1.0, 2.0));
        pose1->transformBy(make_shared<Pose2D>(1.0, 0.0, 0.0));
        //pose2->transformBy(make_shared<Pose2D>(-1.0, 1.0, 2.0));

        output=motion_planner->update(t, pose);
        output1=motion_planner1->update(t, pose1);
        output2=motion_planner2->update(t, pose2);
        
        
        
        //cout<<t<<": " <<pose->getTranslation()->x()<<","<<pose->getTranslation()->y()<<","<<pose->getRotation()->getDegrees()<<","<<motion_planner->mError->getTranslation()->x()<<","<<motion_planner->mError->getTranslation()->y()<<","<<motion_planner->mError->getRotation()->getDegrees()<<","<<output->left_velocity<<","<<output->right_velocity<<","<<output->left_acceleration<<","<<output->right_acceleration<<","<<output->left_feedforward_voltage<<","<<output->right_feedforward_voltage<<","<<endl;
        //cout<<t<<": " <<pose1->getTranslation()->x()<<","<<pose1->getTranslation()->y()<<","<<pose1->getRotation()->getDegrees()<<","<<motion_planner1->mError->getTranslation()->x()<<","<<motion_planner1->mError->getTranslation()->y()<<","<<motion_planner1->mError->getRotation()->getDegrees()<<","<<output1->left_velocity<<","<<output1->right_velocity<<","<<output1->left_acceleration<<","<<output1->right_acceleration<<","<<output1->left_feedforward_voltage<<","<<output1->right_feedforward_voltage<<","<<motion_planner1->toPlannerCSV()<<","<<endl;//<<endl;//
        //cout<<t<<": " <<pose2->getTranslation()->x()<<","<<pose2->getTranslation()->y()<<","<<pose2->getRotation()->getDegrees()<<","<<motion_planner2->mError->getTranslation()->x()<<","<<motion_planner2->mError->getTranslation()->y()<<","<<motion_planner2->mError->getRotation()->getDegrees()<<","<<output2->left_velocity<<","<<output2->right_velocity<<","<<output2->left_acceleration<<","<<output2->right_acceleration<<","<<output2->left_feedforward_voltage<<","<<output2->right_feedforward_voltage<<","<<motion_planner2->toPlannerCSV()<<","<<endl;//<<endl;//
        t+=.09;

    }
    
}

TEST_F(DriveMotionPlannerTest, testTimingConstraints){
    EXPECT_NEAR(motion_planner->constraints->max_velocity, 10.0, kEpsilon);
    

    //MinMaxAcceleration
    shared_ptr<TimingConstraint::MinMaxAcceleration> min_max= make_shared<TimingConstraint::MinMaxAcceleration>();

    EXPECT_NEAR(-1E100, min_max->min_acceleration(), kEpsilon);
    EXPECT_NEAR(1E100, min_max->max_acceleration(), kEpsilon);

    shared_ptr<TimedState> state = make_shared<TimedState>(make_shared<Pose2DWithCurvature>(), 5.0, 2.0, 1.24);

    EXPECT_NEAR(1E100, motion_planner->constraints->CAC->getMaxVelocity(state->state()), kEpsilon);  
    //EXPECT_NEAR(9.0, motion_planner->constraints->DDDC->getMaxVelocity(state->state()), kEpsilon); 


}

TEST_F(DriveMotionPlannerTest, testTimedViewSample){
    //EXPECT_NEAR(3.19328, timed_view->last_interpolant(), 1E-4);
    EXPECT_NEAR(0.0, timed_view->first_interpolant(), 1E-4);
    
    
    shared_ptr<TrajectorySamplePoint>sample_state= timed_view->sample(0.0);
    
    
    
    EXPECT_NEAR(sample_state->state()->state()->getTranslation()->x(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->state()->getTranslation()->y(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->state()->getRotation()->getDegrees(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->velocity(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->acceleration(), 120.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->t(), 0.0, kEpsilon);

    sample_state= timed_view->sample(-5.0);
    EXPECT_NEAR(sample_state->state()->state()->getTranslation()->x(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->state()->getTranslation()->y(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->state()->getRotation()->getDegrees(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->velocity(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->acceleration(), 120.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->t(), 0.0, kEpsilon);

    //sample_state= timed_view->sample(3.193285);
    //EXPECT_NEAR(sample_state->state()->state()->getTranslation()->x(), 240.0, kEpsilon);
    //EXPECT_NEAR(sample_state->state()->state()->getTranslation()->y(), -36.0, kEpsilon);
    //EXPECT_NEAR(sample_state->state()->state()->getRotation()->getDegrees(), 0.0, kEpsilon);
    //EXPECT_NEAR(sample_state->state()->velocity(), 0.0, kEpsilon);
    //EXPECT_NEAR(sample_state->state()->acceleration(), -120.0, kEpsilon);
    //EXPECT_NEAR(sample_state->state()->t(), 3.19328, kEpsilon);


    
     sample_state= timed_view->sample(2.5);
    /*
    EXPECT_NEAR(sample_state->state()->state()->getTranslation()->x(), 240.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->state()->getTranslation()->y(), -36.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->state()->getRotation()->getDegrees(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->velocity(), 0.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->acceleration(), -120.0, kEpsilon);
    EXPECT_NEAR(sample_state->state()->t(), 3.19328, kEpsilon);
    */
}

TEST_F(DriveMotionPlannerTest, TestTrajectoryIterator){

    //initial conditions
    EXPECT_NEAR(0.0, trajectory_iterator->getProgress(), kEpsilon);
    //EXPECT_NEAR(3.19328, trajectory_iterator->getRemainingProgress(), kEpsilon);
    EXPECT_NEAR(points.at(0)->acceleration(), trajectory_iterator->getState()->acceleration(), kEpsilon);
    EXPECT_NEAR(points.at(0)->velocity(), trajectory_iterator->getState()->velocity(), kEpsilon);
    EXPECT_NEAR(points.at(0)->t(), trajectory_iterator->getState()->t(), kEpsilon);    
    EXPECT_NEAR(points.at(0)->state()->getTranslation()->x(), trajectory_iterator->getState()->state()->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(points.at(0)->state()->getTranslation()->y(), trajectory_iterator->getState()->state()->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(points.at(0)->state()->getRotation()->getDegrees(), trajectory_iterator->getState()->state()->getRotation()->getDegrees(), kEpsilon);

    //advance past the end
    trajectory_iterator->advance(5.0);
    EXPECT_NEAR(points.at(points.size()-1)->acceleration(), trajectory_iterator->getState()->acceleration(), kEpsilon);
    EXPECT_NEAR(points.at(points.size()-1)->velocity(), trajectory_iterator->getState()->velocity(), kEpsilon);
    EXPECT_NEAR(points.at(points.size()-1)->t(), trajectory_iterator->getState()->t(), kEpsilon);    
    EXPECT_NEAR(points.at(points.size()-1)->state()->getTranslation()->x(), trajectory_iterator->getState()->state()->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(points.at(points.size()-1)->state()->getTranslation()->y(), trajectory_iterator->getState()->state()->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(points.at(points.size()-1)->state()->getRotation()->getDegrees(), trajectory_iterator->getState()->state()->getRotation()->getDegrees(), kEpsilon);
    EXPECT_TRUE(trajectory_iterator->isDone());

    //advance past beginning
    trajectory_iterator->advance(-10.0);    
    EXPECT_NEAR(points.at(0)->acceleration(), trajectory_iterator->getState()->acceleration(), kEpsilon);
    EXPECT_NEAR(points.at(0)->velocity(), trajectory_iterator->getState()->velocity(), kEpsilon);
    EXPECT_NEAR(points.at(0)->t(), trajectory_iterator->getState()->t(), kEpsilon);    
    EXPECT_NEAR(points.at(0)->state()->getTranslation()->x(), trajectory_iterator->getState()->state()->getTranslation()->x(), kEpsilon);
    EXPECT_NEAR(points.at(0)->state()->getTranslation()->y(), trajectory_iterator->getState()->state()->getTranslation()->y(), kEpsilon);
    EXPECT_NEAR(points.at(0)->state()->getRotation()->getDegrees(), trajectory_iterator->getState()->state()->getRotation()->getDegrees(), kEpsilon);

    EXPECT_FALSE(trajectory_iterator->isDone());
}


TEST_F(DriveMotionPlannerTest, TestDifferentialDrive){
    cout<<motion_planner->mModel->Mass()<<","<<motion_planner->mModel->Moi()<<","<<motion_planner->mModel->Effective_wheelbase_radius()<<","<<motion_planner->mModel->Wheel_radius()<<","<<motion_planner->mModel->Angular_drag()<<","<<endl;
}

TEST_F(DriveMotionPlannerTest, TestCheckTrajectory){
    EXPECT_NEAR(points.at(0)->velocity(), 0.0, kEpsilon);
    EXPECT_NEAR(points.at(points.size()-1)->velocity(), 0.0, kEpsilon);

    for (int i=0; i<points.size()-1; i++){
        shared_ptr<TimedState> state= points.at(i);
        EXPECT_TRUE(state->velocity()-kEpsilon<=motion_planner->constraints->getMaxVelocity(state->state()));
        EXPECT_TRUE(state->acceleration()-kEpsilon<=motion_planner->constraints->getMinMaxAcceleration(state->state(), state->velocity())->max_acceleration());
        EXPECT_TRUE(state->acceleration()+kEpsilon>=motion_planner->constraints->getMinMaxAcceleration(state->state(), state->velocity())->min_acceleration());
        if(i>0){
            shared_ptr<TimedState> prev_state=points.at(i-1);
            EXPECT_NEAR(state->velocity(), prev_state->velocity()+(state->t()-prev_state->t())*prev_state->acceleration(), kEpsilon);
        }
    }
}