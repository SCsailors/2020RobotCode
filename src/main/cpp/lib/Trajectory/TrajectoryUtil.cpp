/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/TrajectoryUtil.h"

TrajectoryUtil::TrajectoryUtil() {}


vector<shared_ptr<TimedState>> TrajectoryUtil::mirrorTimed(vector<shared_ptr<TimedState>> trajectory){
    vector<shared_ptr<TimedState>> waypoints;
    for(shared_ptr<TimedState> state: trajectory){
        shared_ptr<TimedState> waypoint = make_shared<TimedState>(state->state()->mirror(), state->t(), state->velocity(), state->acceleration());
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

vector<shared_ptr<Pose2DWithCurvature>> TrajectoryUtil::trajectoryFromSplineWaypoints(vector<shared_ptr<Pose2D>> waypoints){
    //frc::SmartDashboard::PutString("creating splines", "creating");
    return trajectoryFromSplineWaypoints(waypoints, maxDx, maxDy, maxDTheta);
}

vector<shared_ptr<Pose2DWithCurvature>> TrajectoryUtil::trajectoryFromSplineWaypoints(vector<shared_ptr<Pose2D>> waypoints, double maxDx, double maxDy, double maxDTheta){
    int j=0;
    vector<shared_ptr<Pose2DWithCurvature>> pose;
    for(i=1; i <=waypoints.size()-1; ++i){ //fixed, I think
        j++;
        //cout<<"j= "<<j<<endl;
        //cout<<"i= "<<i<<endl;
        pose.push_back(make_shared<Pose2DWithCurvature>());
        shared_ptr<QuinticHermiteSpline> Quintic= make_shared<QuinticHermiteSpline>(waypoints.at(i-1), waypoints.at(i));
        mQuintic.push_back(Quintic);

    }
    return trajectoryFromSplines(mQuintic, maxDx, maxDy, maxDTheta);
}

vector<shared_ptr<Pose2DWithCurvature>> TrajectoryUtil::trajectoryFromSplines(vector<shared_ptr<QuinticHermiteSpline>> splines, double maxDx, double maxDy, double maxDTheta){
    shared_ptr<SplineGenerator> splineGen= make_shared<SplineGenerator>();
    return splineGen->parameterizeSplines( mQuintic);
}