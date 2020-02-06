/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/TimedView.h"
#include "lib/Trajectory/TrajectoryPoint.h"
#include "Robot.h"

TimedView::TimedView(vector<shared_ptr<TimedState>> trajectory){
    trajectory_=trajectory;
    start_t_=trajectory_.at(0)->t();
    end_t_=trajectory_.at(trajectory_.size()-1)->t();
}

double TimedView::first_interpolant(){
    return start_t_;
}

double TimedView::last_interpolant(){
    return end_t_;
}

vector<shared_ptr<TimedState>> TimedView::trajectory(){
    return trajectory_;
}

shared_ptr<TrajectorySamplePoint> TimedView::sample(double t){
    if(t>=end_t_){
        shared_ptr<TrajectorySamplePoint> atEnd=make_shared<TrajectorySamplePoint>(trajectory_.at(trajectory_.size()-1), trajectory_.size()-1);
        return atEnd;
        
    } else if(t<=start_t_){
        shared_ptr<TrajectorySamplePoint> atBegin=make_shared<TrajectorySamplePoint>(trajectory_.at(0.0), 0);
        return atBegin;
    }
    
    for(int i=1; i<trajectory_.size(); i++){ //fixed already, I think
        shared_ptr<TrajectorySamplePoint> s =make_shared<TrajectorySamplePoint>(trajectory_.at(i), i);
        if( s->state()->t()>=t){
            shared_ptr<TrajectorySamplePoint> prev_s=make_shared<TrajectorySamplePoint>(trajectory_.at(i-1), i-1);
            if(Robot::util.epsilonEquals(s->state()->t(), prev_s->state()->t())){
                shared_ptr<TrajectorySamplePoint> st =make_shared<TrajectorySamplePoint>(s->state(), s->index_floor());
                return st;
            }
            shared_ptr<TrajectorySamplePoint> point=make_shared<TrajectorySamplePoint>(prev_s->state()->interpolate(s->state(), (t-prev_s->state()->t())/(s->state()->t()-prev_s->state()->t())), i-1, i); 
            return point;
        }
    }
    
}
