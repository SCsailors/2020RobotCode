/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Geometry/Rotation2D.h"
#include "Robot.h"
#include <cmath>
using namespace std;

Rotation2D::Rotation2D(){
    this->cos_angle_=1.0;
    this->sin_angle_=0.0;
    this->normalize_=false;
}

Rotation2D::Rotation2D(double x, double y, bool normalize) {
    this->x_=x;
    this->y_=y;
    this->normalize_=normalize;
    if(normalize_){
        magnitude=hypot(x_,y_);
        if (magnitude>kEpsilon){
            sin_angle_=y_/magnitude;
            cos_angle_=x_/magnitude;
        }else{
            sin_angle_=0;
            cos_angle_=y_;
        }
    }else{
        cos_angle_=x_;
        sin_angle_=y_;
    }
}

shared_ptr<Rotation2D> Rotation2D::fromRadians(double radians){
    shared_ptr<Rotation2D> temp = make_shared<Rotation2D>(std::cos(radians), std::sin(radians), false);
    return temp;
}

shared_ptr<Rotation2D> Rotation2D::fromDegrees(double degrees){
    return fromRadians(3.14159265/180.0*degrees);
}

double Rotation2D::cos(){
    return cos_angle_;
}

double Rotation2D::sin(){
    return sin_angle_;
}

shared_ptr<Rotation2D> Rotation2D::rotateBy(shared_ptr<Rotation2D> other){
    shared_ptr<Rotation2D> rotation3= make_shared<Rotation2D>(cos_angle_*other->cos()-sin_angle_*other->sin(), cos_angle_*other->sin()+sin_angle_*other->cos(), true);
    
    return rotation3;
}

double Rotation2D::getRadians(){
    return atan2(sin_angle_, cos_angle_);
}

double Rotation2D::getDegrees(){
    return 180.0/3.14159265*getRadians();
}

shared_ptr<Rotation2D> Rotation2D::inverse(){
    shared_ptr<Rotation2D> rotation4= make_shared<Rotation2D>(cos_angle_, -sin_angle_, false);
    return rotation4;
}

shared_ptr<Rotation2D> Rotation2D::normal(){
    shared_ptr<Rotation2D> normal=make_shared<Rotation2D>(cos_angle_, -sin_angle_, false);
    return normal;
}

shared_ptr<Rotation2D> Rotation2D::interpolate(shared_ptr<Rotation2D> other, double x){
    shared_ptr<Rotation2D> inter = make_shared<Rotation2D>(sin(), cos(), false);
    if (x<=0.0){
        
        return inter;
    } else if( x>=1.0){
        return other;
    }
    double angle_diff=inter->inverse()->rotateBy(other)->getRadians();
    return inter->rotateBy(other->fromRadians(angle_diff*x));
}

bool Rotation2D::isParallel(shared_ptr<Rotation2D> other){
    shared_ptr<Translation2D> temp = make_shared<Translation2D>();
    return Robot::util.epsilonEquals(temp->cross(toTranslation(), other->toTranslation()), 0.0);
}

shared_ptr<Translation2D> Rotation2D::toTranslation(){
    return make_shared<Translation2D>(cos_angle_, sin_angle_);
}