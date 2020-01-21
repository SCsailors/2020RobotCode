/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Geometry/Twist2D.h"

Twist2D::Twist2D(){
    dx=0.0;
    dy=0.0;
    dtheta=0.0;
}

Twist2D::Twist2D(double dx_, double dy_, double dtheta_) {
    dx=dx_;
    dy=dy_;
    dtheta=dtheta_;
}

shared_ptr<Twist2D> Twist2D::scaled(double scale){
    shared_ptr<Twist2D> scaled=make_shared<Twist2D>(dx*scale, dy*scale, dtheta*scale);
    return scaled;
}

double Twist2D::norm(){
    if(dy==0.0){
        return fabs(dx);
    }
    return hypot(dx, dy);
}