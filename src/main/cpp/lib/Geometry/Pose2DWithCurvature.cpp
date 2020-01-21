/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Geometry/Pose2DWithCurvature.h"

Pose2DWithCurvature::Pose2DWithCurvature() {
    pose_=make_shared<Pose2D>();
    curvature_=0.0;
    dcurvature_ds_=0.0;

}

Pose2DWithCurvature::Pose2DWithCurvature(shared_ptr<Pose2D> pose, double curvature, double dcurvature_ds){
    pose_=pose;
    curvature_=curvature;
    dcurvature_ds_=dcurvature_ds;
}

shared_ptr<Pose2D> Pose2DWithCurvature::getPose(){
    return pose_;
}

shared_ptr<Rotation2D> Pose2DWithCurvature::getRotation(){
    return getPose()->getRotation();
}

shared_ptr<Translation2D> Pose2DWithCurvature::getTranslation(){
    return getPose()->getTranslation();
}

shared_ptr<Pose2DWithCurvature> Pose2DWithCurvature::transformBy(shared_ptr<Pose2D> transform){
    shared_ptr<Pose2DWithCurvature> poseC1= make_shared<Pose2DWithCurvature>(getPose()->transformBy(transform), getCurvature(), getDCurvatureDs());
    return poseC1;
}

double Pose2DWithCurvature::getCurvature(){
    return curvature_;
}

double Pose2DWithCurvature::getDCurvatureDs(){
    return dcurvature_ds_;
}

double Pose2DWithCurvature::getDistance(shared_ptr<Pose2DWithCurvature> other){
    return getPose()->distance(other->getPose());
}

shared_ptr<Pose2DWithCurvature> Pose2DWithCurvature::interpolate(shared_ptr<Pose2DWithCurvature> other, double x){
    shared_ptr<Pose2DWithCurvature> Pose= make_shared<Pose2DWithCurvature>(
        getPose()->interpolate(other->getPose(), x),
        util->interpolate(getCurvature(), other->getCurvature(), x),
        util->interpolate(getDCurvatureDs(), other->getDCurvatureDs(), x));
    return Pose;
}

shared_ptr<Pose2DWithCurvature> Pose2DWithCurvature::mirror(){
    return make_shared<Pose2DWithCurvature>(getPose()->mirror(), -getCurvature(), -getDCurvatureDs());
}