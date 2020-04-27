/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Splines/QuinticHermiteSpline.h"
#include <iostream>

QuinticHermiteSpline::QuinticHermiteSpline(shared_ptr<Pose2D> p0, shared_ptr<Pose2D> p1){
double scale=1.2* p0->getTranslation()->distance(p1->getTranslation());

x0=p0->getTranslation()->x();
x1=p1->getTranslation()->x();
dx0=p0->getRotation()->cos()*scale;
dx1=p1->getRotation()->cos()*scale;
ddx0=0;
ddx1=0;

y0=p0->getTranslation()->y();
y1=p1->getTranslation()->y();
dy0=p0->getRotation()->sin()*scale;
dy1=p1->getRotation()->sin()*scale;
ddy0=0;
ddy1=0;

computeCoefficients();
}

QuinticHermiteSpline::QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1, 
                        double y0, double y1, double dy0, double dy1, double ddy0, double ddy1) {
this->x0=x0;
this->x1=x1;
this->dx0=dx0;
this->dx1=dx1;
this->ddx0=ddx0;
this->ddx1=ddx1;

this->y0=y0;
this->y1=y1;
this->dy0=dy0;
this->dy1=dy1;
this->ddy0=ddy0;
this->ddy1=ddy1;

computeCoefficients();
}

void QuinticHermiteSpline::computeCoefficients(){
    Ax=-6*x0-3*dx0-.5*ddx0+.5*ddx1-3*dx1+6*x1;
    Bx=15*x0+8*dx0+1.5*ddx0-ddx1+7*dx1-15*x1;
    Cx=-10*x0-6*dx0-1.5*ddx0+.5*ddx1-4*dx1+10*x1;
    Dx=.5*ddx0;
    Ex=dx0;
    Fx=x0;

    Ay=-6*y0-3*dy0-.5*ddy0+.5*ddy1-3*dy1+6*y1;
    By=15*y0+8*dy0+1.5*ddy0-ddy1+7*dy1-15*y1;
    Cy=-10*y0-6*dy0-1.5*ddy0+.5*ddy1-4*dy1+10*y1;
    Dy=.5*ddy0;
    Ey=dy0;
    Fy=y0;
}

shared_ptr<Pose2D> QuinticHermiteSpline::getStartPose(){
    shared_ptr<Translation2D> t = make_shared<Translation2D>(x0, y0);
    shared_ptr<Rotation2D> r = make_shared<Rotation2D>(dx0, dy0, true);
    return make_shared<Pose2D>(t, r);
}

shared_ptr<Pose2D> QuinticHermiteSpline::getEndPose(){
    shared_ptr<Translation2D> t = make_shared<Translation2D>(x1, y1);
    shared_ptr<Rotation2D> r = make_shared<Rotation2D>(dx1, dy1, true);
    return make_shared<Pose2D>(t, r);
}

shared_ptr<Translation2D> QuinticHermiteSpline::getPoint(double t){
    double x= Ax*t*t*t*t*t+Bx*t*t*t*t+Cx*t*t*t+Dx*t*t+Ex*t+Fx;
    double y= Ay*t*t*t*t*t+By*t*t*t*t+Cy*t*t*t+Dy*t*t+Ey*t+Fy;
    return make_shared<Translation2D>(x,y);
}

double QuinticHermiteSpline::dx(double t){
    return 5*Ax*t*t*t*t+4*Bx*t*t*t+3*Cx*t*t+Dx*t+Ex;
}

double QuinticHermiteSpline::dy(double t){
    return 5*Ay*t*t*t*t+4*By*t*t*t+3*Cy*t*t+Dy*t+Ey;
}

double QuinticHermiteSpline::ddx(double t){
    return 20*Ax*t*t*t+12*Bx*t*t+6*Cx*t+2*Dx;
}

double QuinticHermiteSpline::ddy(double t){
    return 20*Ay*t*t*t+12*By*t*t+6*Cy*t+2*Dy;
}

double QuinticHermiteSpline::dddx(double t){
    return 60*Ax*t*t+24*Bx*t+6*Cx;
}

double QuinticHermiteSpline::dddy(double t){
    return 60*Ay*t*t+24*By*t+6*Cy;
}

double QuinticHermiteSpline::getVelocity(double t){
    return hypot(dx(t), dy(t));
}

double QuinticHermiteSpline::getCurvature(double t){
    return (dx(t)*ddy(t)-ddx(t)*dy(t))/ ((dx(t)*dx(t)+dy(t)*dy(t))*sqrt((dx(t)*dx(t)+dy(t)*dy(t))));
}

double QuinticHermiteSpline::getDCurvature(double t){
    double dx2dy2=(dx(t)*dx(t)+dy(t)*dy(t));
    double num=(dx(t)*dddy(t)-dddx(t)*dy(t))*dx2dy2-3*(dx(t)*ddy(t)-ddx(t)*dy(t))*(dx(t)*ddx(t)+dy(t)*ddy(t));
    return num/(dx2dy2*dx2dy2*sqrt(dx2dy2));
}

double QuinticHermiteSpline::dCurvature2(double t){
    double dx2dy2=(dx(t)*dx(t)+dy(t)*dy(t));
    double num=(dx(t)*dddy(t)-dddx(t)*dy(t))*dx2dy2-3*(dx(t)*ddy(t)-ddx(t)*dy(t))*(dx(t)*ddx(t)+dy(t)*ddy(t));
    return num*num/(dx2dy2*dx2dy2*dx2dy2*dx2dy2*dx2dy2);
}

shared_ptr<Rotation2D> QuinticHermiteSpline::getHeading(double t){
    return make_shared<Rotation2D>(dx(t), dy(t), true);
}

shared_ptr<Pose2D> QuinticHermiteSpline::getPose2D(double t){
    return make_shared<Pose2D>(getPoint(t), getHeading(t));
}

shared_ptr<Pose2DWithCurvature> QuinticHermiteSpline::getPose2DWithCurvature(double t){
    return make_shared<Pose2DWithCurvature>(getPose2D(t), getCurvature(t), getDCurvature(t)/getVelocity(t));
}

double QuinticHermiteSpline::sumDCurvature2(){
    double dt=1.0/kSamples;
    double sum=0.0;
    for(double t=0.0; t<1.0; t+=dt){
        sum+=(dt*dCurvature2(t));
    }
    return sum;
}

double QuinticHermiteSpline::sumDCurvature2(vector<shared_ptr<QuinticHermiteSpline>> splines){
    double sum=0.0;
    for(shared_ptr<QuinticHermiteSpline> s : splines){
        sum+= s->sumDCurvature2();
    }
    return sum;
}

double QuinticHermiteSpline::optimizeSpline(vector<shared_ptr<QuinticHermiteSpline>> splines){
    int count = 0;
    double prev= sumDCurvature2(splines);
    while (count <kMaxIterations){
        runOptimizationIteration(splines);
        double current=sumDCurvature2(splines);
        //std::cout<<current<<std::endl;
        if (prev-current<kMinDelta){
            return current;
        }
        prev=current;
        count++;
    }
    return prev;
}
//runs a single optimization iteration
void QuinticHermiteSpline::runOptimizationIteration(vector<shared_ptr<QuinticHermiteSpline>> splines){
    //can't optimize anything with less than 2 splines
    if (splines.size()<=1){
        return;
    }

    vector<shared_ptr<QuinticHermiteSpline::ControlPoint>> controlPoint;
    double magnitude=0.0;

    for (int i=0; i<splines.size()-2; ++i){
        //don't try to optimize collinear points
        
        if(splines.at(i)->getStartPose()->isColinear(splines.at(i+1)->getStartPose())||splines.at(i)->getEndPose()->isColinear(splines.at(i+1)->getEndPose()) ){
            continue;
        }
        double original=sumDCurvature2(splines);
        shared_ptr<QuinticHermiteSpline> temp;
        shared_ptr<QuinticHermiteSpline> temp1;

        temp= splines.at(i);
        temp1= splines.at(i+1);

        shared_ptr<QuinticHermiteSpline::ControlPoint> point=make_shared<QuinticHermiteSpline::ControlPoint>();
        controlPoint.push_back(point);

        //calculate partial derivatives of sumDCurvature2

        splines.at(i)= make_shared<QuinticHermiteSpline>(temp->x0, temp->x1, temp->dx0, temp->dx1, temp->ddx0, temp->ddx1+kEpsilon, temp->y0, temp->y1, temp->dy0, temp->dy1, temp->ddy0, temp->ddy1);
        splines.at(i+1)= make_shared<QuinticHermiteSpline>(temp1->x0, temp1->x1, temp1->dx0, temp1->dx1, temp1->ddx0, temp1->ddx1+kEpsilon, temp1->y0, temp1->y1, temp1->dy0, temp1->dy1, temp1->ddy0, temp1->ddy1);
        controlPoint.at(i)->ddx=(sumDCurvature2(splines)-original)/kEpsilon;

        splines.at(i)= make_shared<QuinticHermiteSpline>(temp->x0, temp->x1, temp->dx0, temp->dx1, temp->ddx0, temp->ddx1, temp->y0, temp->y1, temp->dy0, temp->dy1, temp->ddy0, temp->ddy1+kEpsilon);
        splines.at(i+1)= make_shared<QuinticHermiteSpline>(temp1->x0, temp1->x1, temp1->dx0, temp1->dx1, temp1->ddx0, temp1->ddx1, temp1->y0, temp1->y1, temp1->dy0, temp1->dy1, temp1->ddy0+kEpsilon, temp1->ddy1);
        controlPoint.at(i)->ddy=(sumDCurvature2(splines)-original)/kEpsilon;

        splines.at(i)= temp;
        splines.at(i+1)= temp1;
        magnitude+= controlPoint.at(i)->ddx*controlPoint.at(i)->ddx+controlPoint.at(i)->ddy*controlPoint.at(i)->ddy;

        magnitude=sqrt(magnitude);

        //minimize along the direction of the gradient
        shared_ptr<Translation2D> p1;
        shared_ptr<Translation2D> p2;
        shared_ptr<Translation2D> p3;

        p2=make_shared<Translation2D>(0.0, sumDCurvature2(splines)); //middle point is at the current location

        for (int i =0; i<splines.size()-2; ++i){
            
            if(splines.at(i)->getStartPose()->isColinear(splines.at(i+1)->getStartPose())||splines.at(i)->getEndPose()->isColinear(splines.at(i+1)->getEndPose()) ){
                continue;
            }   

            //normalize to step size
            controlPoint.at(i)->ddx*= kStepSize/magnitude;
            controlPoint.at(i)->ddy*= kStepSize/magnitude;
            //move opposite the gradient by step size amount
            splines.at(i)->ddx1-=controlPoint.at(i)->ddx;
            splines.at(i)->ddy1-=controlPoint.at(i)->ddy;
            splines.at(i+1)->ddx0-=controlPoint.at(i)->ddx;
            splines.at(i+1)->ddy0-=controlPoint.at(i)->ddy;

            //recompute the spline's coefficients to account for new second derivatives
            splines.at(i)->computeCoefficients();
            splines.at(i+1)->computeCoefficients();
        }

        p1=make_shared<Translation2D>(-kStepSize, sumDCurvature2(splines));

        for(int i=0; i<splines.size()-2; ++i){
            if(splines.at(i)->getStartPose()->isColinear(splines.at(i+1)->getStartPose())||splines.at(i)->getEndPose()->isColinear(splines.at(i+1)->getEndPose()) ){
                continue;
            }

            //move along the gradient by 2 times the step size amount
            splines.at(i)->ddx1+=2*controlPoint.at(i)->ddx;
            splines.at(i)->ddy1+=2*controlPoint.at(i)->ddy;
            splines.at(i+1)->ddx0+=2*controlPoint.at(i)->ddx;
            splines.at(i+1)->ddy0+=2*controlPoint.at(i)->ddy; 
        
            splines.at(i)->computeCoefficients();
            splines.at(i+1)->computeCoefficients();
        }

        p3=make_shared<Translation2D>(kStepSize, sumDCurvature2(splines));

        double stepSize= fitParabola(p1, p2, p3); //approximate step size to minimize sumDCurvature2 along the gradient

        for (int i=0; i<splines.size()-2; ++i){
            // TODO add colinearity
            //move by the step size calculated by the parabola fit

            controlPoint.at(i)->ddx*=1+stepSize/kStepSize;
            controlPoint.at(i)->ddy*=1+stepSize/kStepSize;
        
            splines.at(i)->ddx1+=controlPoint.at(i)->ddx;
            splines.at(i)->ddy1+=controlPoint.at(i)->ddy;
            splines.at(i+1)->ddx0+=controlPoint.at(i)->ddx;
            splines.at(i+1)->ddy0+=controlPoint.at(i)->ddy; 
        
            splines.at(i)->computeCoefficients();
            splines.at(i+1)->computeCoefficients();
        }
    }
}


//returns x-coordinate of the vertex of the parabola
double QuinticHermiteSpline::fitParabola(shared_ptr<Translation2D> p1, shared_ptr<Translation2D> p2, shared_ptr<Translation2D> p3){
    double A=(p3->x()*(p2->y()-p1->y())+p2->x()*(p1->y()-p3->y())+p1->x()*(p3->y()-p2->y()));
    double B=(p3->x()*p3->x()*(p1->y()-p2->y())+p2->x()*p2->x()*(p3->y()-p1->y())+p1->x()*p1->x()*(p2->y()-p3->y()));
    return -B/(2*A);
}