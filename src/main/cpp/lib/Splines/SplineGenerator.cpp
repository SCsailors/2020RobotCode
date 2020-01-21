/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
#include "lib/Splines/SplineGenerator.h"

SplineGenerator::SplineGenerator() {}

std::vector<std::shared_ptr<Pose2DWithCurvature>> SplineGenerator::parameterizeSpline(shared_ptr<QuinticHermiteSpline> s, double maxDx, double maxDy, double maxDTheta, double t0, double t1){
    
    
    rv.push_back(s->getPose2DWithCurvature(0.0));
    //cout<< rv.at(rv.size()-1)->getTranslation()->x()<<","<<rv.at(rv.size()-1)->getTranslation()->y()<<","<<rv.at(rv.size()-1)->getRotation()->getDegrees()<<","<<0.0<<","<<endl;
    
    b++;
    //cout<<"b="<<b<<","<<endl;
    dt=(t1-t0);
    t=0;
    for(t; t<t1; t+=dt/kMinSampleSize){
        
        getSegmentArc(s, t, t+dt/kMinSampleSize, maxDx, maxDy, maxDTheta);
        //cout<<"Size of rv: "<<rv.size()<<endl;
    }
    return rv;
    

}
//only use these two for 
std::vector<std::shared_ptr<Pose2DWithCurvature>> SplineGenerator::parameterizeSpline(shared_ptr<QuinticHermiteSpline> s){
    rv.clear();
    return parameterizeSpline(s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
}
vector<shared_ptr<Pose2DWithCurvature>> SplineGenerator::parameterizeSplines(vector<shared_ptr<QuinticHermiteSpline>> splines){
     rv.clear();
    return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
}


vector<shared_ptr<Pose2DWithCurvature>> SplineGenerator::parameterizeSplines(vector<shared_ptr<QuinticHermiteSpline>> splines, double maxDx, double maxDy, double maxDTheta){
     if(splines.size()==0){return rv;}
     
     //cout<< rv.at(rv.size()-1)->getTranslation()->x()<<","<<rv.at(rv.size()-1)->getTranslation()->y()<<","<<rv.at(rv.size()-1)->getRotation()->getDegrees()<<","<<0.0<<","<<endl;
     int j=0;
     for(auto spline : splines){
         samples=parameterizeSpline(spline, maxDx,maxDy, maxDTheta, 0.0, 1.0);
         rv.pop_back();
     }
     rv.push_back(splines.at(splines.size()-1)->getPose2DWithCurvature(1.0));
     return rv;
 }
 
void SplineGenerator::getSegmentArc(shared_ptr<QuinticHermiteSpline> s, double t0, double t1, double maxDx, double maxDy, double maxDTheta){
    
    p0=s->getPoint(t0);
    p1=s->getPoint(t1);

    r0=s->getHeading(t0);
    r1=s->getHeading(t1);

    shared_ptr<Translation2D> transformation1= make_shared<Translation2D>(p0,p1);
    shared_ptr<Pose2D> transformation= make_shared<Pose2D>(transformation1->rotateBy(r0->inverse()), r1->rotateBy(r0->inverse()));
    shared_ptr<Twist2D> twist1=transformation->log(transformation);
    
    if (twist1->dy> maxDy || twist1->dx>maxDx || twist1->dtheta> maxDTheta){
        getSegmentArc( s,  t0,  (t0+t1)/2.,  maxDx,  maxDy,  maxDTheta);
        getSegmentArc( s,  (t0+t1)/2.,  t1,  maxDx,  maxDy,  maxDTheta);
    
    }else{
    
    rv.push_back(s->getPose2DWithCurvature(t1));
    //cout<< rv.at(rv.size()-1)->getTranslation()->x()<<","<<rv.at(rv.size()-1)->getTranslation()->y()<<","<<rv.at(rv.size()-1)->getRotation()->getDegrees()<<","<<t1<<","<<endl;
    }
}

void SplineGenerator::getSegmentArc(shared_ptr<QuinticHermiteSpline> s, double t0, double t1){
    getSegmentArc(s, t0, t1, kMaxDX, kMaxDY, kMaxDTheta);
}


