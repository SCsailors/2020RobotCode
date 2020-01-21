/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Geometry/Translation2D.h"
#include "lib/Geometry/Rotation2D.h"

using namespace std;

Translation2D::Translation2D(){
    this->x_=0.0;
    this->y_=0.0;
}

Translation2D::Translation2D(shared_ptr<Translation2D> start, shared_ptr<Translation2D> end){
    this->x_=end->x_-start->x_;
    this->y_=end->y_-start->y_;
}

Translation2D::Translation2D(double x, double y) {
    this->x_=x;
    this->y_=y;
}

double Translation2D::norm(){
    return hypot(x_, y_);
}
double Translation2D::norm(double x, double y){
    return hypot(x,y);
}

double Translation2D::x(){
    return x_;
}

double Translation2D::y(){
    return y_;
}

shared_ptr<Translation2D> Translation2D::translateBy(shared_ptr<Translation2D> other){
    shared_ptr<Translation2D> translation1=make_shared<Translation2D>(x_+other->x(), y_+other->y());
    
    return translation1;
}

shared_ptr<Translation2D> Translation2D::rotateBy( shared_ptr<Rotation2D> rotation){
    shared_ptr<Translation2D> translation2= make_shared<Translation2D>(x_*rotation->cos()-y_*rotation->sin(), x_*rotation->sin()+y_*rotation->cos());
    return translation2;
}

shared_ptr<Translation2D> Translation2D::inverse(){
    shared_ptr<Translation2D> translation3=make_shared<Translation2D>(-x_, -y_);
    
    return translation3;
}


double Translation2D::distance(shared_ptr<Translation2D> other){
    
    
    return inverse()->translateBy(other)->norm() ;
}

shared_ptr<Translation2D> Translation2D::interpolate(shared_ptr<Translation2D> other, double x){
    shared_ptr<Translation2D> interp=make_shared<Translation2D>( x_, y_);
    if(x<=0.0){
        return interp;
    }else if(x>= 1.0){
        return other;
    }
    return extrapolate(other, x);
}

shared_ptr<Translation2D> Translation2D::extrapolate(shared_ptr<Translation2D> other, double x){
    shared_ptr<Translation2D> extrap=make_shared<Translation2D>(x*(other->x()-x_)+x_, x*(other->y()-y_)+y_);
    return extrap;
}

double Translation2D::cross(shared_ptr<Translation2D> a, shared_ptr<Translation2D> b){
    return a->x()*b->y()-a->y()*b->x();
}