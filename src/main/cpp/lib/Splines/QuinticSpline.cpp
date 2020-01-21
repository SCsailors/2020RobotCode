/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Splines/QuinticSpline.h"

QuinticSpline::QuinticSpline(){
    scale=1.0;
    x0=0.0;
    x1=0.0;
    dx0=0.0;
    dx1=0.0;
    ddx0=0.0;
    ddx1=0.0;

    y0=0.0;
    y1=0.0;
    dy0=0.0;
    dy1=0.0;
    ddy0=0.0;
    ddy1=0.0;
}
QuinticSpline::QuinticSpline(shared_ptr<Pose2D> p0, shared_ptr<Pose2D> p1, bool first) {
    scale=1.2*p0->getTranslation()->distance(p1->getTranslation());

    x0=p0->getTranslation()->x();
    x1=p1->getTranslation()->x(); 
    dx0=p0->getRotation()->cos()*scale;
    dx1=p1->getRotation()->cos()*scale;
    if (first){
        ddx0=0;
    }else{
        ddx0=p0->getTranslation()->x();
    }
    ddx1= p1->getTranslation()->x();

    y0=p0->getTranslation()->y();
    y1=p1->getTranslation()->y();
    dy0=p0->getRotation()->sin()*scale;
    dy1=p1->getRotation()->sin()*scale;
    if (first){
        ddy0=0;
    }else{
        ddy0=p0->getTranslation()->x();
    }
    ddy1= p1->getTranslation()->y();

}

double QuinticSpline::h0(double t){
    return 1-10*t*t*t+15*t*t*t*t-6*t*t*t*t*t;
}

double QuinticSpline::h1(double t){
    return t-6*t*t*t+8*t*t*t*t-3*t*t*t*t*t;
}

double QuinticSpline::h2(double t){
    return .5*t*t-1.5*t*t*t+1.5*t*t*t*t-.5*t*t*t*t*t;
}

double QuinticSpline::h3(double t){
    return .5*t*t*t-t*t*t*t+.5*t*t*t*t*t;
}

double QuinticSpline::h4(double t){
    return -4*t*t*t+7*t*t*t*t-3*t*t*t*t*t;
}

double QuinticSpline::h5(double t){
    return 10*t*t*t-15*t*t*t*t+6*t*t*t*t*t;
}

double QuinticSpline::dh0(double t){
    return -30*t*t+60*t*t*t-30*t*t*t*t;
}

double QuinticSpline::dh1(double t){
    return 1-18*t*t+32*t*t*t-15*t*t*t*t;
}

double QuinticSpline::dh2(double t){
    return t-4.5*t*t+6*t*t*t-2.5*t*t*t*t;
}

double QuinticSpline::dh3(double t){
    return 1.5*t*t-4*t*t*t+2.5*t*t*t*t;
}

double QuinticSpline::dh4(double t){
    return -12*t*t+28*t*t*t-15*t*t*t*t;
}

double QuinticSpline::dh5(double t){
    return 30*t*t-60*t*t*t+30*t*t*t*t;
}

double QuinticSpline::ddh0(double t){
    return -60*t+180*t*t-120*t*t*t;
}

double QuinticSpline::ddh1(double t){
    return -36*t+96*t*t-60*t*t*t;
}

double QuinticSpline::ddh2(double t){
    return 1-9*t+18*t*t-10*t*t*t;
}

double QuinticSpline::ddh3(double t){
    return 3*t-12*t*t+10*t*t*t;
}

double QuinticSpline::ddh4(double t){
    return -24*t+84*t*t-60*t*t*t;
}

double QuinticSpline::ddh5(double t){
    return 60*t-180*t*t+120*t*t*t;
}

double QuinticSpline::dddh0(double t){
    return -60+360*t-360*t*t;
}

double QuinticSpline::dddh1(double t){
    return -36+192*t-180*t*t;
}

double QuinticSpline::dddh2(double t){
    return 9+36*t-30*t*t;
}

double QuinticSpline::dddh3(double t){
    return 3-24*t+30*t*t;
}

double QuinticSpline::dddh4(double t){
    return -24+168*t-180*t*t;
}

double QuinticSpline::dddh5(double t){
    return 60-360*t+360*t*t;
}

shared_ptr<Pose2D> QuinticSpline::getStartPose(){
    shared_ptr<Translation2D> translation5=make_shared<Translation2D>(x0, y0);
    shared_ptr<Rotation2D> rotation4= make_shared<Rotation2D>(dx0, dy0, true);
    shared_ptr<Pose2D> pose3=make_shared<Pose2D>(translation5,rotation4);
    
    return pose3;
}

shared_ptr<Pose2D> QuinticSpline::getEndPose(){
    shared_ptr<Translation2D> translation4=make_shared<Translation2D>(x1, y1);
    shared_ptr<Rotation2D> rotation5= make_shared<Rotation2D>(dx1, dy1, true);
    shared_ptr<Pose2D> pose4= make_shared<Pose2D>(translation4, rotation5);
    
    return pose4;
    
}

shared_ptr<Translation2D> QuinticSpline::getPoint(double t){
    double x= h0(t)*x0+h1(t)*dx0+h2(t)*ddx0+h3(t)*x1+h4(t)*dx1+h5(t)*ddx1;
    double y= h0(t)*y0+h1(t)*dy0+h2(t)*ddy0+h3(t)*y1+h4(t)*dy1+h5(t)*ddy1;
    shared_ptr<Translation2D> trans_=make_shared<Translation2D>(x,y);
    return trans_;
}

double QuinticSpline::dx(double t){
    return dh0(t)*x0+dh1(t)*dx0+dh2(t)*ddx0+dh3(t)*x1+dh4(t)*dx1+dh5(t)*ddx1;
}

double QuinticSpline::dy(double t){
    return dh0(t)*y0+dh1(t)*dy0+dh2(t)*ddy0+dh3(t)*y1+dh4(t)*dy1+dh5(t)*ddy1;
}

double QuinticSpline::ddx(double t){
    return ddh0(t)*x0+ddh1(t)*dx0+ddh2(t)*ddx0+ddh3(t)*x1+ddh4(t)*dx1+ddh5(t)*ddx1;
}

double QuinticSpline::ddy(double t){
    return ddh0(t)*y0+ddh1(t)*dy0+ddh2(t)*ddy0+ddh3(t)*y1+ddh4(t)*dy1+ddh5(t)*ddy1;
}

double QuinticSpline::dddx(double t){
    return dddh0(t)*x0+dddh1(t)*dx0+dddh2(t)*ddx0+dddh3(t)*x1+dddh4(t)*dx1+dddh5(t)*ddx1;
}

double QuinticSpline::dddy(double t){
    return dddh0(t)*y0+dddh1(t)*dy0+dddh2(t)*ddy0+dddh3(t)*y1+dddh4(t)*dy1+dddh5(t)*ddy1;
}

double QuinticSpline::getVelocity(double t){
    return std::hypot(dx(t), dy(t));
}

double QuinticSpline::getCurvature(double t){
    double curvature=(dx(t)*ddy(t)-ddx(t)*dy(t))/((dx(t)*dx(t)+dy(t)*dy(t))*std::sqrt((dx(t)*dx(t)+dy(t)*dy(t))));
    if(util->epsilonEquals(dx(t), 0.0) || util->epsilonEquals(dy(t), 0.0)){
        return 0.0;// TODO: check if it should be zero or +-NAN
    } else{
        return curvature;
    }
}

double QuinticSpline::getDCurvature(double t){
    double dx2dy2=(dx(t)*dx(t)+dy(t)*dy(t));
    double num=(dx(t)*dddy(t)-dddx(t)*dy(t))*dx2dy2-3*(dx(t)*ddy(t)-ddx(t)*dy(t))*(dx(t)*ddx(t)+dy(t)*ddy(t));
    if(util->epsilonEquals(dx2dy2, 0.0)){
        return 0.0;
    }
    return num/(dx2dy2*dx2dy2*std::sqrt(dx2dy2));
}

shared_ptr<Rotation2D> QuinticSpline::getHeading(double t){
    shared_ptr<Rotation2D> rotation1= make_shared<Rotation2D>(dx(t), dy(t), true);
    return rotation1;
}

shared_ptr<Pose2D> QuinticSpline::getPose2D(double t){
    shared_ptr<Pose2D> pose5=make_shared<Pose2D>(getPoint(t), getHeading(t));
    return pose5;
}

shared_ptr<Pose2DWithCurvature> QuinticSpline::getPose2DWithCurvature(double t){
    shared_ptr<Pose2DWithCurvature> poseC2= make_shared<Pose2DWithCurvature>(getPose2D(t), getCurvature(t), getDCurvature(t)/getVelocity(t));
    return poseC2;
}