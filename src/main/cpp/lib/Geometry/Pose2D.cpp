/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Geometry/Pose2D.h"
#include "Robot.h"
#include <memory>
using namespace std;

Pose2D::Pose2D(){
    translation_= make_shared<Translation2D>();
    rotation_= make_shared<Rotation2D>();
}


Pose2D::Pose2D(shared_ptr<Translation2D> translation, shared_ptr<Rotation2D> rotation) {
    translation_=translation;
    rotation_=rotation;
    
}

Pose2D::Pose2D(double x, double y, double degrees_theta){
    translation_=make_shared<Translation2D>(x,y);
    shared_ptr<Rotation2D> temp=make_shared<Rotation2D>();
    rotation_=temp->fromDegrees(degrees_theta);
}

shared_ptr<Pose2D> Pose2D::exp(shared_ptr<Twist2D> delta){
    double sin_theta=sin(delta->dtheta);
    double cos_theta=cos(delta->dtheta);
    if (abs(delta->dtheta)<kEps){
        s=1.0-1.0/6.0*delta->dtheta*delta->dtheta;
        c=.5*delta->dtheta;
    }
    else{
        s=sin_theta/delta->dtheta;
        c= (1.0-cos_theta)/delta->dtheta;
    }
    shared_ptr<Translation2D> trans_=make_shared<Translation2D>(delta->dx*s-delta->dy*c, delta->dx*c+delta->dy*s);
    shared_ptr<Rotation2D> rotation2=make_shared<Rotation2D>(cos_theta, sin_theta, false);
    shared_ptr<Pose2D> pose1= make_shared<Pose2D>(trans_, rotation2);
    return pose1;
}  

shared_ptr<Twist2D> Pose2D::log(shared_ptr<Pose2D> transform){
    dtheta=transform->getRotation()->getRadians();
    half_dtheta=.5*dtheta;
    cos_minus_one=transform->getRotation()->cos()-1.0;
    if (abs(cos_minus_one)<kEps){
        halfdtheta_by_tan_halfdtheta=1.0-1.0/12.0*dtheta*dtheta;
    } else{
        halfdtheta_by_tan_halfdtheta=-(half_dtheta*transform->getRotation()->sin())/cos_minus_one;
    }
    shared_ptr<Rotation2D> rotation3 = make_shared<Rotation2D>(halfdtheta_by_tan_halfdtheta, -half_dtheta, false);
    shared_ptr<Translation2D> translation8(transform->getTranslation()->rotateBy(rotation3));
    shared_ptr<Twist2D> twist1 = make_shared<Twist2D>(translation8->x(), translation8->y(), dtheta);
    return twist1;
}

shared_ptr<Pose2D> Pose2D::transformBy(shared_ptr<Pose2D> other){
    shared_ptr<Pose2D> pose2= make_shared<Pose2D>(translation_->translateBy(other->translation_->rotateBy(rotation_)),rotation_->rotateBy(other->rotation_));
    
    return pose2;
}

shared_ptr<Translation2D> Pose2D::getTranslation(){
    return translation_;
}

shared_ptr<Rotation2D> Pose2D::getRotation(){
    return rotation_;
}

shared_ptr<Pose2D> Pose2D::inverse(){
    shared_ptr<Rotation2D> rotation_inverted= rotation_->inverse();
    shared_ptr<Pose2D> inverse = make_shared<Pose2D>(translation_->inverse()->rotateBy(rotation_inverted), rotation_inverted);
    return inverse;
}

shared_ptr<Pose2D> Pose2D::normal(){
    shared_ptr<Pose2D> normal= make_shared<Pose2D>(translation_, rotation_->normal());
    return normal;
}

shared_ptr<Pose2D> Pose2D::interpolate(shared_ptr<Pose2D> other, double x){
    if( x<=0.0){
        shared_ptr<Pose2D> Pose=make_shared<Pose2D>(getTranslation(), getRotation());
        return Pose;
    } else if (x>=1.0){
        shared_ptr<Pose2D> Pose=make_shared<Pose2D>(other->getTranslation(), other->getRotation());
        return Pose;
    }
    shared_ptr<Twist2D> twist=log(inverse()->transformBy(other));
    return transformBy(exp(twist->scaled(x)));
}

double Pose2D::distance(shared_ptr<Pose2D> other){
    return log(inverse()->transformBy(other))->norm(); 
}

shared_ptr<Pose2D> Pose2D::mirror(){
    shared_ptr<Pose2D> tmp = make_shared<Pose2D>(getTranslation()->x(), -getTranslation()->y(), getRotation()->inverse()->getDegrees());
    return tmp;
}

bool Pose2D::isColinear(shared_ptr<Pose2D> other){
    if(!getRotation()->isParallel(other->getRotation())){
        return false;
    }
    shared_ptr<Twist2D> twist = log(inverse()->transformBy(other));
    return (Robot::util.epsilonEquals(twist->dy, 0.0)&&Robot::util.epsilonEquals(twist->dtheta, 0.0));
}